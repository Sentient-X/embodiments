//! Video probe and commit (UVC 1.5 §4.3.1.1): the streaming parameters negotiated on the
//! VideoStreaming interface before its bandwidth is selected. The structure is 26 bytes in
//! UVC 1.0, 34 in 1.1 and 48 in 1.5; every field this rig reads is in the first 34.

use std::io;

use crate::usb::{CLASS_INTERFACE_IN, CLASS_INTERFACE_OUT, Control};

pub const VS_PROBE_CONTROL: u8 = 0x01;
pub const VS_COMMIT_CONTROL: u8 = 0x02;
pub const SET_CUR: u8 = 0x01;
pub const GET_CUR: u8 = 0x81;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Probe {
    pub hint: u16,
    pub format_index: u8,
    pub frame_index: u8,
    pub frame_interval: u32,
    pub key_frame_rate: u16,
    pub p_frame_rate: u16,
    pub comp_quality: u16,
    pub comp_window_size: u16,
    pub delay: u16,
    pub max_video_frame_size: u32,
    pub max_payload_transfer_size: u32,
    pub clock_frequency: u32,
    pub framing_info: u8,
    pub preferred_version: u8,
    pub min_version: u8,
    pub max_version: u8,
}

/// The probe length for a camera's bcdUVC.
pub fn length(uvc_version: u16) -> usize {
    match uvc_version {
        0..=0x0100 => 26,
        0x0101..=0x0149 => 34,
        _ => 48,
    }
}

impl Probe {
    pub fn encode(&self, length: usize) -> Vec<u8> {
        let mut out = Vec::with_capacity(48);
        out.extend_from_slice(&self.hint.to_le_bytes());
        out.push(self.format_index);
        out.push(self.frame_index);
        out.extend_from_slice(&self.frame_interval.to_le_bytes());
        out.extend_from_slice(&self.key_frame_rate.to_le_bytes());
        out.extend_from_slice(&self.p_frame_rate.to_le_bytes());
        out.extend_from_slice(&self.comp_quality.to_le_bytes());
        out.extend_from_slice(&self.comp_window_size.to_le_bytes());
        out.extend_from_slice(&self.delay.to_le_bytes());
        out.extend_from_slice(&self.max_video_frame_size.to_le_bytes());
        out.extend_from_slice(&self.max_payload_transfer_size.to_le_bytes());
        out.extend_from_slice(&self.clock_frequency.to_le_bytes());
        out.push(self.framing_info);
        out.push(self.preferred_version);
        out.push(self.min_version);
        out.push(self.max_version);
        out.resize(length, 0);
        out
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() < 26 {
            return Err(format!("probe control is {} bytes, need 26", bytes.len()));
        }
        let u16_at = |at: usize| u16::from_le_bytes([bytes[at], bytes[at + 1]]);
        let u32_at = |at: usize| {
            u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
        };
        let mut probe = Probe {
            hint: u16_at(0),
            format_index: bytes[2],
            frame_index: bytes[3],
            frame_interval: u32_at(4),
            key_frame_rate: u16_at(8),
            p_frame_rate: u16_at(10),
            comp_quality: u16_at(12),
            comp_window_size: u16_at(14),
            delay: u16_at(16),
            max_video_frame_size: u32_at(18),
            max_payload_transfer_size: u32_at(22),
            ..Probe::default()
        };
        if bytes.len() >= 34 {
            probe.clock_frequency = u32_at(26);
            probe.framing_info = bytes[30];
            probe.preferred_version = bytes[31];
            probe.min_version = bytes[32];
            probe.max_version = bytes[33];
        }
        Ok(probe)
    }
}

/// Negotiate `format_index`/`frame_index`/`interval` on `interface` and commit what the
/// camera answered: SET_CUR probe, GET_CUR probe, check, SET_CUR commit. Returns the
/// committed parameters (the frame size and payload size the bandwidth is chosen from).
pub fn negotiate(
    control: &impl Control,
    interface: u8,
    uvc_version: u16,
    format_index: u8,
    frame_index: u8,
    interval: u32,
) -> io::Result<Probe> {
    let length = length(uvc_version);
    let request = Probe {
        // bmHint: keep dwFrameInterval fixed.
        hint: 0x0001,
        format_index,
        frame_index,
        frame_interval: interval,
        ..Probe::default()
    };
    let index = u16::from(interface);
    control.control_out(
        CLASS_INTERFACE_OUT,
        SET_CUR,
        u16::from(VS_PROBE_CONTROL) << 8,
        index,
        &request.encode(length),
    )?;
    let mut answer = vec![0; length];
    let read = control.control_in(
        CLASS_INTERFACE_IN,
        GET_CUR,
        u16::from(VS_PROBE_CONTROL) << 8,
        index,
        &mut answer,
    )?;
    let answered = Probe::decode(&answer[..read])
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    if answered.format_index != format_index || answered.frame_index != frame_index {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "camera answered format {}/frame {} for {format_index}/{frame_index}",
                answered.format_index, answered.frame_index
            ),
        ));
    }
    if answered.max_payload_transfer_size == 0 || answered.max_video_frame_size == 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "camera answered a zero frame or payload size",
        ));
    }
    control.control_out(
        CLASS_INTERFACE_OUT,
        SET_CUR,
        u16::from(VS_COMMIT_CONTROL) << 8,
        index,
        &answer[..read],
    )?;
    Ok(answered)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn probe_round_trips_at_every_version_length() {
        let probe = Probe {
            hint: 1,
            format_index: 1,
            frame_index: 2,
            frame_interval: 83_263,
            max_video_frame_size: 614_400,
            max_payload_transfer_size: 3_072,
            clock_frequency: 48_000_000,
            ..Probe::default()
        };
        assert_eq!(length(0x0100), 26);
        assert_eq!(length(0x0110), 34);
        assert_eq!(length(0x0150), 48);
        for version in [0x0100, 0x0110, 0x0150] {
            let bytes = probe.encode(length(version));
            assert_eq!(bytes.len(), length(version));
            let back = Probe::decode(&bytes).unwrap();
            assert_eq!(back.frame_interval, 83_263);
            assert_eq!(back.max_payload_transfer_size, 3_072);
            assert_eq!(
                back.clock_frequency,
                if version == 0x0100 { 0 } else { 48_000_000 }
            );
        }
        assert!(Probe::decode(&[0; 25]).is_err());
    }
}
