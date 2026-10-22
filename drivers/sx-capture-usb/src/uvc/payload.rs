//! UVC payload headers (UVC 1.5 §2.4.3.3) and frame reassembly.
//!
//! Every isochronous packet (or bulk transfer) carrying video starts with a header:
//! `bHeaderLength`, `bmHeaderInfo` (FID, EOF, PTS, SCR, RES, STI, ERR, EOH), then the
//! 32-bit PTS — the device clock when capture of this frame began — and the 48-bit SCR — a
//! 32-bit device-clock sample (STC) with the 11-bit USB SOF counter read at the same time.
//! A frame is the payloads between two FID toggles, or up to the payload with EOF set.

const FID: u8 = 0x01;
const EOF: u8 = 0x02;
const HAS_PTS: u8 = 0x04;
const HAS_SCR: u8 = 0x08;
const STILL: u8 = 0x20;
const ERROR: u8 = 0x40;
const END_OF_HEADER: u8 = 0x80;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    pub length: usize,
    pub fid: bool,
    pub end_of_frame: bool,
    pub pts: Option<u32>,
    /// (STC, SOF): device clock and the 11-bit 1 kHz bus frame counter, sampled together.
    pub scr: Option<(u32, u16)>,
    pub still: bool,
    pub error: bool,
}

/// Parse one payload: `Ok(None)` for an empty packet (isochronous streams send many),
/// `Err` for a header that does not fit its packet.
pub fn parse(packet: &[u8]) -> Result<Option<(Header, &[u8])>, String> {
    if packet.is_empty() {
        return Ok(None);
    }
    let length = usize::from(packet[0]);
    if length < 2 || length > packet.len() {
        return Err(format!(
            "payload header length {length} does not fit a {}-byte packet",
            packet.len()
        ));
    }
    let info = packet[1];
    if info & END_OF_HEADER == 0 {
        return Err("payload header without EOH".to_owned());
    }
    let mut at = 2;
    let mut take = |count: usize| -> Result<&[u8], String> {
        let field = packet
            .get(at..at + count)
            .filter(|_| at + count <= length)
            .ok_or_else(|| "payload header shorter than its declared fields".to_owned())?;
        at += count;
        Ok(field)
    };
    let pts = if info & HAS_PTS != 0 {
        let b = take(4)?;
        Some(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    } else {
        None
    };
    let scr = if info & HAS_SCR != 0 {
        let b = take(6)?;
        Some((
            u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            u16::from_le_bytes([b[4], b[5]]) & 0x07ff,
        ))
    } else {
        None
    };
    Ok(Some((
        Header {
            length,
            fid: info & FID != 0,
            end_of_frame: info & EOF != 0,
            pts,
            scr,
            still: info & STILL != 0,
            error: info & ERROR != 0,
        },
        &packet[length..],
    )))
}

/// One reassembled frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Assembled {
    pub bytes: Vec<u8>,
    pub pts: Option<u32>,
    /// Counts every frame boundary the stream showed, delivered or not, so a gap in it is
    /// a frame the camera sent and the rig lost (as a V4L2 sequence gap is).
    pub sequence: u32,
    /// Host time of the first payload (the earliest the frame could have been known).
    pub first_payload_ns: i64,
}

/// Why a frame boundary produced no frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dropped {
    /// The camera flagged an error in one of its payloads.
    DeviceError,
    /// The JPEG lacked its SOI or EOI marker: a payload was lost on the bus.
    Truncated,
    /// The frame outgrew the negotiated maximum frame size.
    Oversized,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    Frame(Assembled),
    Dropped { sequence: u32, why: Dropped },
}

pub struct Assembler {
    max_frame_size: usize,
    jpeg: bool,
    fid: Option<bool>,
    bytes: Vec<u8>,
    pts: Option<u32>,
    error: bool,
    oversized: bool,
    first_payload_ns: i64,
    next_sequence: u32,
    open: bool,
    /// The FID of a frame its EOF closed: payloads still carrying it belong to that frame
    /// (cameras keep sending header-only payloads until the next frame toggles FID), so
    /// they open nothing — as Linux uvcvideo drops them "out of sync".
    ended_fid: Option<bool>,
}

impl Assembler {
    pub fn new(max_frame_size: usize, jpeg: bool) -> Self {
        Self {
            max_frame_size,
            jpeg,
            fid: None,
            bytes: Vec::with_capacity(max_frame_size),
            pts: None,
            error: false,
            oversized: false,
            first_payload_ns: 0,
            next_sequence: 0,
            open: false,
            ended_fid: None,
        }
    }

    fn close(&mut self, events: &mut Vec<Event>) {
        if !self.open {
            return;
        }
        self.open = false;
        let sequence = self.next_sequence;
        self.next_sequence = self.next_sequence.wrapping_add(1);
        let bytes = std::mem::replace(&mut self.bytes, Vec::with_capacity(self.max_frame_size));
        let why = if self.error {
            Some(Dropped::DeviceError)
        } else if self.oversized {
            Some(Dropped::Oversized)
        } else if self.jpeg
            && (bytes.len() < 4 || bytes[..2] != [0xff, 0xd8] || !ends_with_eoi(&bytes))
        {
            Some(Dropped::Truncated)
        } else {
            None
        };
        events.push(match why {
            Some(why) => Event::Dropped { sequence, why },
            None => Event::Frame(Assembled {
                bytes,
                pts: self.pts,
                sequence,
                first_payload_ns: self.first_payload_ns,
            }),
        });
        self.pts = None;
        self.error = false;
        self.oversized = false;
    }

    /// Feed one payload received at host time `host_ns`.
    pub fn push(&mut self, header: &Header, data: &[u8], host_ns: i64, events: &mut Vec<Event>) {
        if header.still {
            return;
        }
        if self.fid.is_some_and(|fid| fid != header.fid) {
            self.close(events);
            self.ended_fid = None;
        }
        self.fid = Some(header.fid);
        if !self.open && self.ended_fid == Some(header.fid) {
            return;
        }
        if !self.open {
            if data.is_empty() && !header.end_of_frame {
                // A header-only payload carries no frame bytes; do not open a frame on it.
                if header.pts.is_none() {
                    return;
                }
            }
            self.open = true;
            self.first_payload_ns = host_ns;
        }
        if self.pts.is_none() {
            self.pts = header.pts;
        }
        self.error |= header.error;
        if self.bytes.len() + data.len() > self.max_frame_size {
            self.oversized = true;
        } else {
            self.bytes.extend_from_slice(data);
        }
        if header.end_of_frame {
            self.close(events);
            self.ended_fid = Some(header.fid);
        }
    }
}

/// A JPEG ends at EOI; some cameras pad the last payload with zeros after it.
fn ends_with_eoi(bytes: &[u8]) -> bool {
    let trimmed = bytes
        .iter()
        .rposition(|b| *b != 0)
        .map_or(&bytes[..0], |end| &bytes[..=end]);
    trimmed.ends_with(&[0xff, 0xd9])
}

/// Build a payload header (tests and the simulated camera).
pub fn header_bytes(fid: bool, eof: bool, pts: Option<u32>, scr: Option<(u32, u16)>) -> Vec<u8> {
    let mut info = END_OF_HEADER;
    if fid {
        info |= FID;
    }
    if eof {
        info |= EOF;
    }
    let mut out = vec![0, 0];
    if let Some(pts) = pts {
        info |= HAS_PTS;
        out.extend_from_slice(&pts.to_le_bytes());
    }
    if let Some((stc, sof)) = scr {
        info |= HAS_SCR;
        out.extend_from_slice(&stc.to_le_bytes());
        out.extend_from_slice(&(sof & 0x07ff).to_le_bytes());
    }
    out[0] = out.len() as u8;
    out[1] = info;
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_fields_parse_as_the_spec_lays_them_out() {
        // A 12-byte header as Linux uvcvideo logs them: 0c 8d <pts> <stc> <sof>.
        let packet = [
            0x0c, 0x8d, 0x78, 0x56, 0x34, 0x12, 0xf0, 0xde, 0xbc, 0x9a, 0xff, 0x0f, 0xaa,
        ];
        let (header, data) = parse(&packet).unwrap().unwrap();
        assert!(header.fid && !header.end_of_frame && !header.error);
        assert_eq!(header.pts, Some(0x1234_5678));
        // SOF keeps only its 11 bits.
        assert_eq!(header.scr, Some((0x9abc_def0, 0x07ff)));
        assert_eq!(data, [0xaa]);
        assert_eq!(parse(&[]).unwrap(), None);
        assert!(parse(&[0x0c, 0x8c, 0]).is_err());
        assert!(parse(&[0x02, 0x0c]).is_err(), "no EOH");
        assert!(
            parse(&[0x06, 0x8c, 1, 2, 3, 4]).is_err(),
            "PTS+SCR in 6 bytes"
        );
    }

    fn feed(assembler: &mut Assembler, packets: &[Vec<u8>]) -> Vec<Event> {
        let mut events = Vec::new();
        for (index, packet) in packets.iter().enumerate() {
            if let Some((header, data)) = parse(packet).unwrap() {
                assembler.push(&header, data, index as i64, &mut events);
            }
        }
        events
    }

    fn payload(fid: bool, eof: bool, pts: u32, data: &[u8]) -> Vec<u8> {
        [
            header_bytes(fid, eof, Some(pts), Some((pts + 10, 5))),
            data.to_vec(),
        ]
        .concat()
    }

    /// What the Sonix 05a3:9230 does at 30 fps (bench, Quest 3S): after a frame's EOF it
    /// keeps sending header-only payloads, with PTS and SCR and the same FID, until the next
    /// frame toggles FID. They are not a frame; each used to open one that the toggle then
    /// closed empty, counted as a truncated frame (63 of 57 frames on the bench).
    #[test]
    fn header_only_payloads_after_eof_open_no_frame() {
        let mut assembler = Assembler::new(64, true);
        let mut packets = vec![
            payload(false, false, 100, &[0xff, 0xd8, 1]),
            payload(false, true, 100, &[2, 0xff, 0xd9]),
        ];
        packets.extend((0..40).map(|_| payload(false, false, 100, &[])));
        packets.push(payload(true, false, 200, &[0xff, 0xd8, 3]));
        packets.push(payload(true, true, 200, &[0xff, 0xd9]));
        packets.extend((0..40).map(|_| payload(true, false, 200, &[])));
        packets.push(payload(false, true, 300, &[0xff, 0xd8, 4, 0xff, 0xd9]));
        let events = feed(&mut assembler, &packets);
        let sequences: Vec<u32> = events
            .iter()
            .map(|event| match event {
                Event::Frame(frame) => frame.sequence,
                Event::Dropped { why, .. } => panic!("dropped {why:?}: {events:?}"),
            })
            .collect();
        assert_eq!(sequences, [0, 1, 2]);
    }

    #[test]
    fn frames_close_on_eof_or_fid_toggle_and_bad_ones_are_counted() {
        let mut assembler = Assembler::new(64, true);
        let events = feed(
            &mut assembler,
            &[
                payload(false, false, 100, &[0xff, 0xd8, 1]),
                payload(false, true, 100, &[2, 0xff, 0xd9, 0, 0]),
                // No EOF: closed by the FID toggle that follows.
                payload(true, false, 200, &[0xff, 0xd8, 3, 0xff, 0xd9]),
                // Lost its start on the bus: truncated.
                payload(false, true, 300, &[4, 0xff, 0xd9]),
                // Flagged by the camera.
                [header_bytes(true, false, Some(400), None), vec![0xff, 0xd8]].concat(),
                {
                    let mut p = payload(true, true, 400, &[0xff, 0xd9]);
                    p[1] |= ERROR;
                    p
                },
                payload(
                    false,
                    true,
                    500,
                    &[&[0xff, 0xd8][..], &[0; 70], &[0xff, 0xd9]].concat(),
                ),
            ],
        );
        assert_eq!(events.len(), 5);
        assert!(
            matches!(&events[0], Event::Frame(f) if f.pts == Some(100) && f.sequence == 0
            && f.bytes == [0xff, 0xd8, 1, 2, 0xff, 0xd9, 0, 0])
        );
        assert!(matches!(&events[1], Event::Frame(f) if f.pts == Some(200) && f.sequence == 1));
        assert_eq!(
            events[2],
            Event::Dropped {
                sequence: 2,
                why: Dropped::Truncated
            }
        );
        assert_eq!(
            events[3],
            Event::Dropped {
                sequence: 3,
                why: Dropped::DeviceError
            }
        );
        assert_eq!(
            events[4],
            Event::Dropped {
                sequence: 4,
                why: Dropped::Oversized
            }
        );
    }
}
