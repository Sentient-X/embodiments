//! UVC class-specific descriptors (UVC 1.5 §3.7, §3.9): the VideoControl interface's
//! header (bcdUVC, dwClockFrequency), camera terminal, processing unit and extension units;
//! the VideoStreaming interface's formats, frames, frame intervals and the alternate
//! settings that size its isochronous bandwidth.

use crate::usb::descriptor::{Configuration, DescriptorError, Endpoint, TransferType};

pub const CC_VIDEO: u8 = 0x0e;
pub const SC_VIDEOCONTROL: u8 = 0x01;
pub const SC_VIDEOSTREAMING: u8 = 0x02;
const CS_INTERFACE: u8 = 0x24;
const VC_HEADER: u8 = 0x01;
const VC_INPUT_TERMINAL: u8 = 0x02;
const VC_PROCESSING_UNIT: u8 = 0x05;
const VC_EXTENSION_UNIT: u8 = 0x06;
const ITT_CAMERA: u16 = 0x0201;
const VS_INPUT_HEADER: u8 = 0x01;
const VS_FORMAT_UNCOMPRESSED: u8 = 0x04;
const VS_FRAME_UNCOMPRESSED: u8 = 0x05;
const VS_FORMAT_MJPEG: u8 = 0x06;
const VS_FRAME_MJPEG: u8 = 0x07;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FormatKind {
    Mjpeg,
    Uncompressed,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub index: u8,
    pub width: u16,
    pub height: u16,
    pub max_frame_size: u32,
    pub default_interval: u32,
    /// Discrete intervals in 100 ns units; a continuous range is expanded to its ends.
    pub intervals: Vec<u32>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Format {
    pub index: u8,
    pub kind: FormatKind,
    pub frames: Vec<Frame>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StreamingAlt {
    pub alternate: u8,
    pub endpoint: Endpoint,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UvcCamera {
    pub uvc_version: u16,
    pub control_interface: u8,
    /// The device clock PTS and SCR count in (VC header; deprecated but present in 1.5).
    pub clock_frequency_hz: u32,
    pub camera_terminal: Option<u8>,
    pub processing_unit: Option<u8>,
    pub extension_units: Vec<(u8, [u8; 16])>,
    pub streaming_interface: u8,
    pub formats: Vec<Format>,
    /// Alternate settings with an isochronous IN endpoint, by ascending bandwidth.
    pub iso_alts: Vec<StreamingAlt>,
}

fn malformed(detail: impl Into<String>) -> DescriptorError {
    DescriptorError(detail.into())
}

fn u16_at(bytes: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([bytes[at], bytes[at + 1]])
}

fn u32_at(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
}

impl UvcCamera {
    /// The camera in `configuration`: its first VideoControl interface and the first
    /// VideoStreaming interface that header names.
    pub fn parse(configuration: &Configuration) -> Result<Self, DescriptorError> {
        let control = configuration
            .alt_settings
            .iter()
            .find(|a| a.class == CC_VIDEO && a.subclass == SC_VIDEOCONTROL)
            .ok_or_else(|| malformed("no UVC VideoControl interface"))?;
        let mut camera = UvcCamera {
            uvc_version: 0,
            control_interface: control.number,
            clock_frequency_hz: 0,
            camera_terminal: None,
            processing_unit: None,
            extension_units: Vec::new(),
            streaming_interface: 0,
            formats: Vec::new(),
            iso_alts: Vec::new(),
        };
        let mut streaming = None;
        for record in &control.class_specific {
            if record.len() < 3 || record[1] != CS_INTERFACE {
                continue;
            }
            match record[2] {
                VC_HEADER => {
                    if record.len() < 12 {
                        return Err(malformed("short VC header"));
                    }
                    camera.uvc_version = u16_at(record, 3);
                    camera.clock_frequency_hz = u32_at(record, 7);
                    let count = usize::from(record[11]);
                    streaming = record.get(12..12 + count).and_then(|s| s.first().copied());
                }
                VC_INPUT_TERMINAL if record.len() >= 8 && u16_at(record, 4) == ITT_CAMERA => {
                    camera.camera_terminal = Some(record[3]);
                }
                VC_PROCESSING_UNIT if record.len() >= 4 => {
                    camera.processing_unit = Some(record[3]);
                }
                VC_EXTENSION_UNIT if record.len() >= 20 => {
                    let mut guid = [0; 16];
                    guid.copy_from_slice(&record[4..20]);
                    camera.extension_units.push((record[3], guid));
                }
                _ => {}
            }
        }
        if camera.uvc_version == 0 {
            return Err(malformed("VideoControl interface has no VC header"));
        }
        camera.streaming_interface =
            streaming.ok_or_else(|| malformed("VC header names no VideoStreaming interface"))?;
        for alt in configuration.interface(camera.streaming_interface) {
            if alt.class != CC_VIDEO || alt.subclass != SC_VIDEOSTREAMING {
                return Err(malformed("VC header names a non-streaming interface"));
            }
            if alt.alternate == 0 {
                // The VS descriptors that follow the interface, as Linux uvcvideo reads them.
                camera.formats = parse_formats(&alt.class_specific)?;
            }
            if let Some(endpoint) = alt
                .endpoints
                .iter()
                .find(|e| e.is_in() && e.transfer_type() == TransferType::Isochronous)
            {
                camera.iso_alts.push(StreamingAlt {
                    alternate: alt.alternate,
                    endpoint: endpoint.clone(),
                });
            }
        }
        camera
            .iso_alts
            .sort_by_key(|alt| alt.endpoint.bytes_per_interval());
        if camera.formats.is_empty() {
            return Err(malformed("VideoStreaming interface has no formats"));
        }
        Ok(camera)
    }

    /// The extension unit carrying `guid`, if the camera has one.
    pub fn extension_unit(&self, guid: &[u8; 16]) -> Option<u8> {
        self.extension_units
            .iter()
            .find(|(_, g)| g == guid)
            .map(|(id, _)| *id)
    }

    /// The format/frame for `kind` at exactly `width`×`height`, and of its intervals the one
    /// nearest `interval` (100 ns units).
    pub fn mode(
        &self,
        kind: FormatKind,
        width: u16,
        height: u16,
        interval: u32,
    ) -> Option<(u8, u8, u32)> {
        let format = self.formats.iter().find(|f| f.kind == kind)?;
        let frame = format
            .frames
            .iter()
            .find(|f| f.width == width && f.height == height)?;
        let chosen = frame
            .intervals
            .iter()
            .copied()
            .min_by_key(|i| i.abs_diff(interval))
            .unwrap_or(frame.default_interval);
        Some((format.index, frame.index, chosen))
    }

    /// The alternate setting Linux uvcvideo would pick: the smallest isochronous bandwidth
    /// that carries the committed `dwMaxPayloadTransferSize`.
    pub fn alt_for(&self, payload_size: u32) -> Option<&StreamingAlt> {
        self.iso_alts
            .iter()
            .find(|alt| alt.endpoint.bytes_per_interval() >= payload_size)
    }
}

fn parse_formats(records: &[Vec<u8>]) -> Result<Vec<Format>, DescriptorError> {
    let mut formats: Vec<Format> = Vec::new();
    let mut header = false;
    for record in records {
        if record.len() < 3 || record[1] != CS_INTERFACE {
            continue;
        }
        match record[2] {
            VS_INPUT_HEADER => header = true,
            VS_FORMAT_MJPEG | VS_FORMAT_UNCOMPRESSED => {
                if record.len() < 4 {
                    return Err(malformed("short format descriptor"));
                }
                formats.push(Format {
                    index: record[3],
                    kind: if record[2] == VS_FORMAT_MJPEG {
                        FormatKind::Mjpeg
                    } else {
                        FormatKind::Uncompressed
                    },
                    frames: Vec::new(),
                });
            }
            VS_FRAME_MJPEG | VS_FRAME_UNCOMPRESSED => {
                if record.len() < 26 {
                    return Err(malformed("short frame descriptor"));
                }
                let format = formats
                    .last_mut()
                    .ok_or_else(|| malformed("frame descriptor before its format"))?;
                let kind = if record[2] == VS_FRAME_MJPEG {
                    FormatKind::Mjpeg
                } else {
                    FormatKind::Uncompressed
                };
                if kind != format.kind {
                    return Err(malformed("frame descriptor under the wrong format"));
                }
                let interval_type = usize::from(record[25]);
                let intervals = if interval_type == 0 {
                    if record.len() < 38 {
                        return Err(malformed("short continuous frame descriptor"));
                    }
                    vec![u32_at(record, 26), u32_at(record, 30)]
                } else {
                    if record.len() < 26 + 4 * interval_type {
                        return Err(malformed(
                            "frame descriptor lists more intervals than it holds",
                        ));
                    }
                    (0..interval_type)
                        .map(|i| u32_at(record, 26 + 4 * i))
                        .collect()
                };
                format.frames.push(Frame {
                    index: record[3],
                    width: u16_at(record, 5),
                    height: u16_at(record, 7),
                    max_frame_size: u32_at(record, 17),
                    default_interval: u32_at(record, 21),
                    intervals,
                });
            }
            _ => {}
        }
    }
    if !header {
        return Err(malformed("VideoStreaming interface has no input header"));
    }
    Ok(formats)
}
