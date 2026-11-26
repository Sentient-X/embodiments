//! Raw STS3215 grasp evidence from the normal SYNC READ. The bounded queue
//! drains to a separate evidence file, away from the station log sink.
use std::fs::{File, OpenOptions};
use std::io::{self, BufWriter, Write};
use std::path::PathBuf;
use std::sync::mpsc::{SyncSender, sync_channel};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use sx_embodiment_drivers::{BoundAxis, serial::SerialPort};

use crate::scservo_sdk::{
    GroupSyncRead, ProtocolPacketHandler, SMS_STS_GOAL_POSITION_L, SMS_STS_PRESENT_CURRENT_L,
    SMS_STS_PRESENT_LOAD_L, SMS_STS_PRESENT_POSITION_L, SMS_STS_PRESENT_TEMPERATURE,
    SMS_STS_PRESENT_VOLTAGE, SMS_STS_TORQUE_ENABLE,
};

const RUNTIME_TORQUE_LIMIT: u8 = 48;
const HARDWARE_STATUS: u8 = 65;
const EEPROM_CAP: u8 = 16;
const UNLOAD_CONDITIONS: u8 = 19;
const PROTECTION_CURRENT: u8 = 28;
const POST_TRIP_OUTPUT: u8 = 34;
const OVERLOAD_TIME: u8 = 35;
const OVERLOAD_THRESHOLD: u8 = 36;

pub(crate) const READ_LENGTH: u8 = SMS_STS_PRESENT_CURRENT_L + 2 - SMS_STS_TORQUE_ENABLE;

const BATCH: usize = 16;
const COLUMNS: &str = "bus_id,host_unix_us,goal_raw,position_raw,load_raw,current_raw,voltage_decivolts,temperature_c,runtime_cap_raw,hardware_status,reply_error,torque_enable";

/// A complete packet's registers. All values are raw vendor units. The first
/// column is elapsed host time from the batch's first packet receipt.
type Sample = [u32; 11];

#[derive(Default)]
struct Batch {
    start_unix_us: u64,
    rows: [Sample; BATCH],
    len: usize,
    missing: u32,
    lost_batches: u32,
}

struct Finger {
    id: u8,
    start: Option<Instant>,
    batch: Batch,
}

pub(crate) struct GraspDiagnostics {
    fingers: Vec<Finger>,
    sender: SyncSender<(usize, Batch)>,
}

impl GraspDiagnostics {
    fn new(axes: &[BoundAxis], mut output: BufWriter<File>) -> Option<Self> {
        let (sender, reader) = sync_channel::<(usize, Batch)>(16);
        let ids: Vec<_> = axes.iter().map(|axis| axis.binding.bus_id).collect();
        let fingers = axes
            .iter()
            .map(|axis| {
                Some(Finger {
                    id: u8::try_from(axis.binding.bus_id).ok()?,
                    start: None,
                    batch: Batch::default(),
                })
            })
            .collect::<Option<_>>()?;
        std::thread::Builder::new()
            .name("feetech-grasp".into())
            .spawn(move || {
                while let Ok((index, batch)) = reader.recv() {
                    if write_batch(&mut output, ids[index], &batch).is_err() {
                        break;
                    }
                }
            })
            .ok()?;
        Some(Self { fingers, sender })
    }

    pub(crate) fn observe(&mut self, reader: &GroupSyncRead, end: u8, received: Instant) {
        for (index, finger) in self.fingers.iter_mut().enumerate() {
            let Some(mut row) = sample(reader, end, finger.id) else {
                finger.batch.missing = finger.batch.missing.saturating_add(1);
                continue;
            };
            let from = *finger.start.get_or_insert_with(|| {
                finger.batch.start_unix_us = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .ok()
                    .and_then(|elapsed| elapsed.as_micros().try_into().ok())
                    .unwrap_or(0);
                received
            });
            row[0] = u32::try_from(received.saturating_duration_since(from).as_micros())
                .unwrap_or(u32::MAX);
            finger.batch.rows[finger.batch.len] = row;
            finger.batch.len += 1;
            if finger.batch.len == BATCH || row[8] != 0 || row[9] != 0 {
                Self::publish(&self.sender, index, finger);
            }
        }
    }

    pub(crate) fn flush(&mut self) {
        for (index, finger) in self.fingers.iter_mut().enumerate() {
            if finger.batch.len != 0 || finger.batch.missing != 0 || finger.batch.lost_batches != 0
            {
                Self::publish(&self.sender, index, finger);
            }
        }
    }

    fn publish(sender: &SyncSender<(usize, Batch)>, index: usize, finger: &mut Finger) {
        let batch = std::mem::take(&mut finger.batch);
        let lost = batch.lost_batches;
        finger.start = None;
        if sender.try_send((index, batch)).is_err() {
            finger.batch.lost_batches = lost.saturating_add(1);
        }
    }
}

fn sample(reader: &GroupSyncRead, end: u8, id: u8) -> Option<Sample> {
    let one = |address| reader.get_data1(id, address).map(u32::from);
    let two = |address| reader.get_data2(end, id, address).map(u32::from);
    Some([
        0,
        two(SMS_STS_GOAL_POSITION_L)?,
        two(SMS_STS_PRESENT_POSITION_L)?,
        two(SMS_STS_PRESENT_LOAD_L)?,
        two(SMS_STS_PRESENT_CURRENT_L)?,
        one(SMS_STS_PRESENT_VOLTAGE)?,
        one(SMS_STS_PRESENT_TEMPERATURE)?,
        two(RUNTIME_TORQUE_LIMIT)?,
        one(HARDWARE_STATUS)?,
        u32::from(reader.get_error(id)?),
        one(SMS_STS_TORQUE_ENABLE)?,
    ])
}

/// Snapshot the configured limits, while the chain is still disabled.
/// An unavailable diagnostic read never weakens or overrides the normal admission checks.
fn read_limits<P: SerialPort>(
    handler: &mut ProtocolPacketHandler<P>,
    axes: &[BoundAxis],
    output: &mut impl Write,
) -> io::Result<()> {
    let mut reader = GroupSyncRead::new(EEPROM_CAP, RUNTIME_TORQUE_LIMIT + 2 - EEPROM_CAP);
    for axis in axes {
        if let Ok(id) = u8::try_from(axis.binding.bus_id) {
            reader.add_param(id);
        }
    }
    let _ = reader.tx_rx_packet(handler);
    let end = handler.scs_getend();
    for axis in axes {
        let Ok(id) = u8::try_from(axis.binding.bus_id) else {
            continue;
        };
        writeln!(
            output,
            "# protection id={id} joint={} reply_error={:?} eeprom_cap_raw={:?} runtime_cap_raw={:?} unload_conditions={:?} protection_current_raw={:?} post_trip_output_pct={:?} overload_time_10ms={:?} overload_threshold_pct={:?}",
            axis.joint,
            reader.get_error(id),
            reader.get_data2(end, id, EEPROM_CAP),
            reader.get_data2(end, id, RUNTIME_TORQUE_LIMIT),
            reader.get_data1(id, UNLOAD_CONDITIONS),
            reader.get_data2(end, id, PROTECTION_CURRENT),
            reader.get_data1(id, POST_TRIP_OUTPUT),
            reader.get_data1(id, OVERLOAD_TIME),
            reader.get_data1(id, OVERLOAD_THRESHOLD),
        )?;
    }
    Ok(())
}

fn write_batch(output: &mut impl Write, id: u16, batch: &Batch) -> io::Result<()> {
    if batch.missing != 0 || batch.lost_batches != 0 {
        writeln!(
            output,
            "# gap id={id} before_unix_us={} missing_packets={} dropped_batches={}",
            batch.start_unix_us, batch.missing, batch.lost_batches,
        )?;
    }
    for row in &batch.rows[..batch.len] {
        let wall_us = batch.start_unix_us.saturating_add(u64::from(row[0]));
        write!(output, "{id},{wall_us}")?;
        for value in &row[1..] {
            write!(output, ",{value}")?;
        }
        writeln!(output)?;
    }
    output.flush()
}

fn evidence_file(port: &str) -> Option<BufWriter<File>> {
    if port.is_empty() || !port.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_') {
        return None;
    }
    // systemd sets this to the station service's existing writable StateDirectory.
    let directory = PathBuf::from(std::env::var_os("STATE_DIRECTORY")?).join("evidence/gripper");
    std::fs::create_dir_all(&directory).ok()?;
    let at = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .ok()?
        .as_nanos();
    let path = directory.join(format!("{port}-{at}.csv"));
    OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .ok()
        .map(BufWriter::new)
}

/// Enable the extended reply only after the unpowered chain has demonstrated
/// that every servo can answer it. The caller keeps its position-only reader
/// if this probe or diagnostic worker initialization fails.
pub(crate) fn open<P: SerialPort>(
    handler: &mut ProtocolPacketHandler<P>,
    axes: &[BoundAxis],
) -> Option<(GroupSyncRead, GraspDiagnostics)> {
    let mut output = evidence_file(&axes.first()?.port)?;
    let mut reader = GroupSyncRead::new(SMS_STS_TORQUE_ENABLE, READ_LENGTH);
    for axis in axes {
        if !reader.add_param(u8::try_from(axis.binding.bus_id).ok()?) {
            return None;
        }
    }
    let _ = reader.tx_rx_packet(handler);
    if !axes.iter().all(|axis| {
        u8::try_from(axis.binding.bus_id)
            .ok()
            .and_then(|id| sample(&reader, handler.scs_getend(), id))
            .is_some_and(|row| row[9] == 0)
    }) {
        let _ = writeln!(
            output,
            "# extended reply unavailable; using position-only feedback"
        );
        return None;
    }
    read_limits(handler, axes, &mut output).ok()?;
    writeln!(output, "{COLUMNS}").ok()?;
    output.flush().ok()?;
    Some((reader, GraspDiagnostics::new(axes, output)?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MotorCalibration, sim::SimBus};

    #[test]
    fn the_extended_reply_preserves_raw_load_and_current_and_marks_missing_fingers() {
        let calibration = MotorCalibration {
            drive_mode: false,
            homing_offset: -12,
            range_min: 800,
            range_max: 3200,
        };
        let bus = SimBus::sts3215(&[(3, calibration), (4, calibration)]);
        {
            let mut state = bus.state();
            let regs = state.servos.get_mut(&3).unwrap();
            regs[42..44].copy_from_slice(&2105_u16.to_le_bytes());
            regs[60..62].copy_from_slice(&0x0415_u16.to_le_bytes());
            regs[62] = 121;
            regs[63] = 39;
            regs[65] = 0x20;
            regs[69..71].copy_from_slice(&0x80ab_u16.to_le_bytes());
        }
        let mut handler = ProtocolPacketHandler::new(bus.clone(), 0);
        let mut reader = GroupSyncRead::new(SMS_STS_TORQUE_ENABLE, READ_LENGTH);
        assert!(reader.add_param(3));
        assert!(reader.add_param(4));
        let _ = reader.tx_rx_packet(&mut handler);
        assert_eq!(
            sample(&reader, 0, 3).unwrap(),
            [0, 2105, 2000, 0x0415, 0x80ab, 121, 39, 1000, 0x20, 0, 0]
        );
        bus.kill(4);
        let _ = reader.tx_rx_packet(&mut handler);
        assert!(sample(&reader, 0, 3).is_some());
        assert_eq!(sample(&reader, 0, 4), None);
        assert!(
            bus.state()
                .written
                .iter()
                .all(|p| p[4] == 0x82 && p[5] == 40 && p[6] == 31)
        );
    }

    #[test]
    fn a_batch_keeps_host_time_and_records_gaps() {
        let mut output = Vec::new();
        let mut batch = Batch {
            start_unix_us: 1_000_000,
            len: 1,
            missing: 2,
            lost_batches: 1,
            ..Batch::default()
        };
        batch.rows[0] = [
            10_000, 2105, 2000, 0x0415, 0x80ab, 121, 39, 500, 0x20, 0x20, 1,
        ];
        write_batch(&mut output, 3, &batch).unwrap();
        assert_eq!(
            String::from_utf8(output).unwrap(),
            "# gap id=3 before_unix_us=1000000 missing_packets=2 dropped_batches=1\n3,1010000,2105,2000,1045,32939,121,39,500,32,32,1\n"
        );
    }

    #[test]
    fn a_full_log_queue_drops_the_batch_and_counts_it() {
        let (sender, reader) = sync_channel(1);
        let mut finger = Finger {
            id: 3,
            start: Some(Instant::now()),
            batch: Batch::default(),
        };
        finger.batch.len = 1;
        GraspDiagnostics::publish(&sender, 0, &mut finger);
        finger.batch.len = 1;
        GraspDiagnostics::publish(&sender, 0, &mut finger);
        assert_eq!(finger.batch.lost_batches, 1);
        assert_eq!(reader.try_recv().unwrap().1.len, 1);
        finger.batch.len = 1;
        GraspDiagnostics::publish(&sender, 0, &mut finger);
        assert_eq!(reader.try_recv().unwrap().1.lost_batches, 1);
    }
}
