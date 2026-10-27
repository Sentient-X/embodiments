//! What a robot's host must provide before its drivers can run as they were measured: device
//! access, real-time priority, CPU wake latency, USB power management, kernel modules and
//! interfaces, userspace libraries.
//!
//! Each driver crate declares the needs of one device it opens (`host_needs` beside its
//! opener: `sx_zlg_usbcanfd::host_needs`, `sx_socketcan::host_needs`, the USB serial bridges in
//! `sx-embodiment-bodies`); commissioning, which knows which bridge carries each port, writes
//! their union for the robot as `host-needs.json` ([`HOST_NEEDS_SCHEMA`]) beside
//! `commissioning.json`. The robot host's launcher applies it (udev rules, systemd limits,
//! modules, interface setup); no person or tool hand-installs any of it, and no driver
//! assumes a grant: a need not met is reported once at open and the driver runs as before.
//!
//! The vocabulary is closed ([`HostNeed`]): a new kind of need is a new variant here, so the
//! launcher's handling and every driver's declaration change together.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

/// The schema of a robot's `host-needs.json`.
pub const HOST_NEEDS_SCHEMA: &str = "sx.robot-host-needs/v1";

/// One thing the host must provide.
#[derive(Clone, Debug, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(tag = "need", rename_all = "snake_case", deny_unknown_fields)]
pub enum HostNeed {
    /// Read-write on a USB device's usbfs node for the station's group (a udev rule on the
    /// vendor and product id, narrowed to the serial when one is given): a userspace driver.
    UsbDevice {
        vendor_id: u16,
        product_id: u16,
        serial: Option<String>,
    },
    /// Read-write on a USB serial adapter's tty for the station's group.
    SerialTty {
        vendor_id: u16,
        product_id: u16,
        serial: Option<String>,
    },
    /// USB runtime power management off for the device (`power/control` = `on`): an adapter
    /// that autosuspends between ticks adds its resume time to the next answer.
    UsbAutosuspendOff { vendor_id: u16, product_id: u16 },
    /// `SCHED_FIFO` up to `priority`: `RLIMIT_RTPRIO` of at least it (systemd `LimitRTPRIO=`)
    /// or `CAP_SYS_NICE` for the station's process (`sx-realtime`).
    RealtimePriority { priority: u8 },
    /// Write on `/dev/cpu_dma_latency` for the station's group, so an open bus can hold the
    /// CPU wake latency at zero (`sx-realtime`: on the DGX Spark this took a 14-motor cycle's
    /// host p99 from 1.4-1.9 ms to 71-81 µs).
    CpuWakeLatency,
    /// A kernel module loaded at boot.
    KernelModule { name: String },
    /// A `SocketCAN` interface brought up with these settings at boot
    /// (`ip link set <interface> up type can bitrate <b> restart-ms <r>` and `txqueuelen`).
    CanInterface {
        interface: String,
        bitrate: u32,
        restart_ms: u32,
        txqueue_len: u32,
    },
    /// A userspace shared library installed (by its soname).
    SharedLibrary { name: String },
    /// A crash of the station process leaves a core dump the robot's operators can read
    /// (`systemd-coredump` installed, the unit's `LimitCORE=infinity`): native code (libwebrtc,
    /// vendor drivers) aborts without a Rust backtrace, and a core is the only evidence.
    CoreDumps,
}

/// The pseudo-port under which the station process's own needs are listed, beside the needs of
/// the devices its ports open.
pub const STATION: &str = "station";

/// What the station process itself needs on any robot host, whatever devices it opens.
#[must_use]
pub fn station_needs() -> Vec<HostNeed> {
    vec![HostNeed::CoreDumps]
}

/// A robot's needs: per port as each driver declared them, and their union.
#[derive(Clone, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HostNeeds {
    pub schema: String,
    /// Each commissioned port's needs, by port name.
    pub ports: BTreeMap<String, BTreeSet<HostNeed>>,
    /// Every need once: what the launcher applies.
    pub needs: BTreeSet<HostNeed>,
}

impl HostNeeds {
    /// Needs of no port yet.
    #[must_use]
    pub fn new() -> Self {
        Self {
            schema: HOST_NEEDS_SCHEMA.to_owned(),
            ..Self::default()
        }
    }

    /// Add `port`'s needs, as its driver declared them.
    pub fn add(&mut self, port: &str, needs: impl IntoIterator<Item = HostNeed>) {
        let entry = self.ports.entry(port.to_owned()).or_default();
        for need in needs {
            entry.insert(need.clone());
            self.needs.insert(need);
        }
    }

    /// The highest real-time priority any port needs, if any.
    #[must_use]
    pub fn realtime_priority(&self) -> Option<u8> {
        self.needs
            .iter()
            .filter_map(|need| match need {
                HostNeed::RealtimePriority { priority } => Some(*priority),
                _ => None,
            })
            .max()
    }

    /// Parse a `host-needs.json`.
    ///
    /// # Errors
    ///
    /// Refuses another schema, an unknown need, or a union that is not exactly the ports'.
    pub fn parse(text: &str) -> Result<Self, String> {
        let needs: Self = serde_json::from_str(text).map_err(|error| error.to_string())?;
        if needs.schema != HOST_NEEDS_SCHEMA {
            return Err(format!(
                "host needs schema {:?}, not {HOST_NEEDS_SCHEMA}",
                needs.schema
            ));
        }
        let union: BTreeSet<HostNeed> = needs.ports.values().flatten().cloned().collect();
        if union != needs.needs {
            return Err("host needs: the union is not exactly the ports' needs".to_owned());
        }
        Ok(needs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ports_union_into_one_set_that_round_trips() {
        let mut needs = HostNeeds::new();
        needs.add(
            "can",
            [
                HostNeed::UsbDevice {
                    vendor_id: 0x3068,
                    product_id: 0x0009,
                    serial: Some("A30F".to_owned()),
                },
                HostNeed::RealtimePriority { priority: 80 },
                HostNeed::CpuWakeLatency,
            ],
        );
        needs.add(
            "neck",
            [
                HostNeed::SerialTty {
                    vendor_id: 0x1a86,
                    product_id: 0x55d3,
                    serial: Some("5A7C".to_owned()),
                },
                HostNeed::RealtimePriority { priority: 80 },
            ],
        );
        assert_eq!(needs.needs.len(), 4, "the shared priority once");
        assert_eq!(needs.realtime_priority(), Some(80));
        let text = serde_json::to_string_pretty(&needs).expect("json");
        assert!(text.contains("\"need\": \"cpu_wake_latency\""));
        assert_eq!(HostNeeds::parse(&text).expect("parse"), needs);
        let mut wrong = needs.clone();
        wrong.needs.insert(HostNeed::KernelModule {
            name: "gs_usb".to_owned(),
        });
        let text = serde_json::to_string(&wrong).expect("json");
        assert!(HostNeeds::parse(&text).is_err());
        assert!(
            HostNeeds::parse(r#"{"schema":"sx.robot-host-needs/v1","ports":{"can":[{"need":"magic"}]},"needs":[]}"#)
                .is_err()
        );
    }
}
