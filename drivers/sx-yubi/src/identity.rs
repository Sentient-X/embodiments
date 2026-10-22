//! Factory identities of the rig's USB components, independent of how they are reached:
//! the Sonix wrist camera by its SPI flash UID (read through its extension unit), the
//! ESP32-C6 encoder by the factory MAC its USB serial string carries. The laptop pod reaches
//! the extension unit through `UVCIOC_CTRL_QUERY`; the headset rig through usbfs control
//! transfers. Ports, node names and bus addresses are locations only, never identities.

use std::thread;
use std::time::{Duration, Instant};

/// The supported wrist camera, USB vendor/product.
pub const CAMERA_MODEL: (u16, u16) = (0x32e4, 0x9230);
/// The native ESP32-C6 USB serial/JTAG unit.
pub const ENCODER_MODEL: (u16, u16) = (0x303a, 0x1001);
/// The Sonix extension unit's GUID as it appears in the descriptors (little-endian fields).
pub const CAMERA_XU_GUID: [u8; 16] = [
    0x70, 0x33, 0xf0, 0x28, 0x11, 0x63, 0x2e, 0x4a, 0xba, 0x2c, 0x68, 0x90, 0xeb, 0x33, 0x40, 0x16,
];
/// The extension unit's register-access control selector, 4 bytes wide.
pub const CAMERA_XU_SELECTOR: u8 = 1;
/// UVC request codes used on the extension unit.
pub const UVC_SET_CUR: u8 = 0x01;
pub const UVC_GET_CUR: u8 = 0x81;

#[derive(Debug, PartialEq, Eq)]
pub struct Refused(pub String);

impl std::fmt::Display for Refused {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Refused {}

/// One 4-byte query on the camera's register-access extension control: `UVC_SET_CUR` or
/// `UVC_GET_CUR`.
pub trait Xu {
    type Error: From<Refused>;
    fn query(&self, query: u8, bytes: &mut [u8; 4]) -> Result<(), Self::Error>;
}

/// The camera's SPI flash UID as lowercase hex: three consecutive identical, non-degenerate
/// reads out of at most eight. Only the JEDEC-ID and UID-read opcodes are ever issued, and
/// the SPI select and mode registers are restored whatever happens.
pub fn flash_uid<X: Xu>(xu: &X) -> Result<String, X::Error> {
    let camera = Camera(xu);
    if camera.read(0x101f)? != 0x71 {
        return Err(Refused("Sonix ASIC is not 0x71".into()).into());
    }
    if camera.spi(0x9f)? != [0x5e, 0x60, 0x13] {
        return Err(Refused("flash JEDEC is not 5e6013".into()).into());
    }
    let mut previous = Vec::new();
    let mut consecutive = 0;
    for _ in 0..8 {
        let current = camera.spi(0x4b)?;
        if current
            .iter()
            .collect::<std::collections::BTreeSet<_>>()
            .len()
            < 4
        {
            previous.clear();
            consecutive = 0;
            continue;
        }
        consecutive = if previous == current {
            consecutive + 1
        } else {
            1
        };
        previous = current;
        if consecutive == 3 {
            return Ok(previous.iter().map(|b| format!("{b:02x}")).collect());
        }
    }
    Err(Refused("no three consecutive valid matching flash UID reads".into()).into())
}

struct Camera<'a, X: Xu>(&'a X);

impl<X: Xu> Camera<'_, X> {
    fn read(&self, address: u16) -> Result<u8, X::Error> {
        self.0.query(
            UVC_SET_CUR,
            &mut [address as u8, (address >> 8) as u8, 0, 255],
        )?;
        let mut bytes = [0; 4];
        self.0.query(UVC_GET_CUR, &mut bytes)?;
        Ok(bytes[2])
    }

    fn write(&self, address: u16, value: u8) -> Result<(), X::Error> {
        if ![0x1080, 0x1091, 0x1082, 0x1083, 0x1081].contains(&address) {
            return Err(Refused("non-SPI register write".into()).into());
        }
        self.0.query(
            UVC_SET_CUR,
            &mut [address as u8, (address >> 8) as u8, value, 0],
        )
    }

    fn wait(&self) -> Result<(), X::Error> {
        thread::sleep(Duration::from_millis(5));
        let deadline = Instant::now() + Duration::from_millis(50);
        while Instant::now() < deadline {
            if self.read(0x1084)? & 1 != 0 {
                return Ok(());
            }
            thread::sleep(Duration::from_millis(1));
        }
        Err(Refused("SPI completion timed out".into()).into())
    }

    fn spi(&self, opcode: u8) -> Result<Vec<u8>, X::Error> {
        if ![0x9f, 0x4b].contains(&opcode) {
            return Err(Refused("only flash identification reads permitted".into()).into());
        }
        let selected = self.read(0x1080)?;
        let mode = self.read(0x1091)?;
        if selected != 0 {
            return Err(Refused("SPI bus already selected".into()).into());
        }
        let result = (|| {
            self.write(0x1080, 1)?;
            self.write(0x1091, 0)?;
            self.write(0x1082, opcode)?;
            self.write(0x1081, 1)?;
            self.wait()?;
            if opcode == 0x4b {
                for _ in 0..4 {
                    self.write(0x1082, 0)?;
                    self.write(0x1081, 1)?;
                    self.wait()?;
                }
            }
            (0..if opcode == 0x9f { 3 } else { 16 })
                .map(|_| {
                    self.write(0x1083, 0)?;
                    self.write(0x1081, 2)?;
                    self.wait()?;
                    self.read(0x1083)
                })
                .collect()
        })();
        let deselect = self.write(0x1080, 0);
        let restore = self.write(0x1091, mode);
        deselect?;
        restore?;
        result
    }
}

/// The encoder's identity from its USB serial string: the ESP32's factory MAC, upper-case,
/// as `usb-serial:AA:BB:CC:DD:EE:FF`.
pub fn encoder_identity(serial: &str) -> Result<String, Refused> {
    let mac = serial.trim().to_ascii_uppercase();
    if mac.len() != 17
        || !mac
            .split(':')
            .all(|p| p.len() == 2 && p.bytes().all(|b| b.is_ascii_hexdigit()))
    {
        return Err(Refused("ESP32 USB serial is not a factory MAC".into()));
    }
    Ok(format!("usb-serial:{mac}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::collections::VecDeque;

    #[derive(Debug)]
    enum FakeError {
        Refused(String),
        Unplugged,
    }
    impl From<Refused> for FakeError {
        fn from(value: Refused) -> Self {
            Self::Refused(value.0)
        }
    }
    impl std::fmt::Display for FakeError {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            match self {
                Self::Refused(detail) => f.write_str(detail),
                Self::Unplugged => f.write_str("unplugged"),
            }
        }
    }

    struct Fake {
        selected: RefCell<u16>,
        writes: RefCell<Vec<(u16, u8)>>,
        replies: RefCell<VecDeque<u8>>,
        fail_trigger: RefCell<bool>,
        chip: u8,
        complete: bool,
    }
    impl Fake {
        fn new(uids: Vec<u8>) -> Self {
            Self {
                selected: RefCell::new(0),
                writes: RefCell::new(Vec::new()),
                replies: RefCell::new([vec![0x5e, 0x60, 0x13], uids].concat().into()),
                fail_trigger: RefCell::new(false),
                chip: 0x71,
                complete: true,
            }
        }
    }
    impl Xu for Fake {
        type Error = FakeError;
        fn query(&self, query: u8, b: &mut [u8; 4]) -> Result<(), FakeError> {
            if query == UVC_SET_CUR {
                let address = u16::from_le_bytes([b[0], b[1]]);
                if b[3] == 255 {
                    *self.selected.borrow_mut() = address;
                } else {
                    self.writes.borrow_mut().push((address, b[2]));
                    if address == 0x1081 && self.fail_trigger.replace(false) {
                        return Err(FakeError::Unplugged);
                    }
                }
            } else {
                b[2] = match *self.selected.borrow() {
                    0x101f => self.chip,
                    0x1084 => u8::from(self.complete),
                    0x1091 => 7,
                    0x1083 => self
                        .replies
                        .borrow_mut()
                        .pop_front()
                        .expect("scripted SPI reply"),
                    _ => 0,
                };
            }
            Ok(())
        }
    }

    #[test]
    fn requires_three_matching_nondegenerate_reads_and_only_identification_commands() {
        let uid: Vec<u8> = (0..16).collect();
        let fake = Fake::new([vec![255; 16], (16..32).collect(), uid.repeat(3)].concat());
        assert_eq!(
            flash_uid(&fake).unwrap(),
            "000102030405060708090a0b0c0d0e0f"
        );
        let writes = fake.writes.borrow();
        assert!(
            writes
                .iter()
                .filter(|(a, _)| *a == 0x1082)
                .all(|(_, v)| [0, 0x9f, 0x4b].contains(v))
        );
        assert_eq!(&writes[writes.len() - 2..], &[(0x1080, 0), (0x1091, 7)]);
    }

    #[test]
    fn repeated_stale_bytes_never_become_an_identity() {
        let fake = Fake::new(vec![0; 16 * 8]);
        assert!(
            flash_uid(&fake)
                .unwrap_err()
                .to_string()
                .contains("three consecutive")
        );
    }

    #[test]
    fn unsupported_chip_and_jedec_stop_before_uid_access() {
        let mut fake = Fake::new(Vec::new());
        fake.chip = 0;
        assert!(flash_uid(&fake).is_err());
        assert!(fake.writes.borrow().is_empty());
        let fake = Fake::new(Vec::new());
        *fake.replies.borrow_mut() = vec![0, 0, 0].into();
        assert!(flash_uid(&fake).unwrap_err().to_string().contains("JEDEC"));
        assert!(!fake.writes.borrow().contains(&(0x1082, 0x4b)));
    }

    #[test]
    fn unplug_and_completion_timeout_restore_controller_state() {
        for timeout in [false, true] {
            let mut fake = Fake::new(Vec::new());
            if timeout {
                fake.complete = false;
            } else {
                fake.fail_trigger.replace(true);
            }
            assert!(flash_uid(&fake).is_err());
            let writes = fake.writes.borrow();
            assert_eq!(&writes[writes.len() - 2..], &[(0x1080, 0), (0x1091, 7)]);
        }
    }

    #[test]
    fn encoder_identity_is_the_factory_mac() {
        assert_eq!(
            encoder_identity("f0:f5:bd:01:02:0a\n").unwrap(),
            "usb-serial:F0:F5:BD:01:02:0A"
        );
        for bad in ["", "shared-or-empty", "F0:F5:BD:01:02", "G0:F5:BD:01:02:0A"] {
            assert!(encoder_identity(bad).is_err(), "{bad}");
        }
    }
}
