//! SYNTHETIC descriptor sets, built from the USB 2.0, UVC 1.1 and CDC 1.2 specifications to
//! the shape of the rig's two devices. No real dump of either device has been recorded yet:
//! the ids, the Sonix extension-unit GUID, 640×480 MJPEG at 83 263 × 100 ns and the
//! ESP32-C6 USB Serial/JTAG interface layout are facts the laptop pod relies on; the clock
//! frequency, entity ids other than the extension unit, packet sizes and string indices are
//! plausible choices. The headset's `sx_rig_status` reports the real ones (`devices[].usb`:
//! the YUBI camera is UVC 1.00, a 15 MHz clock, six isochronous alternates).

use sx_yubi::identity::CAMERA_XU_GUID;

fn device(vendor: u16, product: u16, class: [u8; 3]) -> Vec<u8> {
    let mut d = vec![18, 0x01, 0x00, 0x02, class[0], class[1], class[2], 64];
    d.extend_from_slice(&vendor.to_le_bytes());
    d.extend_from_slice(&product.to_le_bytes());
    d.extend_from_slice(&[0x00, 0x01, 1, 2, 3, 1]);
    d
}

fn configuration(interfaces: u8, body: &[u8]) -> Vec<u8> {
    let total = (9 + body.len()) as u16;
    let mut c = vec![9, 0x02];
    c.extend_from_slice(&total.to_le_bytes());
    c.extend_from_slice(&[interfaces, 1, 0, 0x80, 250]);
    c.extend_from_slice(body);
    c
}

fn interface(number: u8, alternate: u8, endpoints: u8, class: [u8; 3]) -> Vec<u8> {
    vec![
        9, 0x04, number, alternate, endpoints, class[0], class[1], class[2], 0,
    ]
}

fn endpoint(address: u8, attributes: u8, size: u16, interval: u8) -> Vec<u8> {
    let mut e = vec![7, 0x05, address, attributes];
    e.extend_from_slice(&size.to_le_bytes());
    e.push(interval);
    e
}

/// The YUBI wrist camera (32e4:9230): VideoControl with camera terminal 1, processing unit 2,
/// the Sonix extension unit 3; VideoStreaming with MJPEG 640×480 (~120 Hz, 60, 30) and
/// 1280×720, isochronous alternates of 128, 1024 and 3×1024 bytes per microframe.
pub fn yubi_camera() -> Vec<u8> {
    let mut vc_body = Vec::new();
    // Camera terminal: AE mode (D1) and exposure time absolute (D3).
    vc_body.extend_from_slice(&[18, 0x24, 0x02, 1, 0x01, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 3]);
    vc_body.extend_from_slice(&[0x0a, 0x00, 0x00]);
    // Processing unit: gain (D9).
    vc_body.extend_from_slice(&[11, 0x24, 0x05, 2, 1, 0, 0, 2, 0x00, 0x02, 0]);
    // Extension unit 3, the Sonix register-access unit.
    let mut xu = vec![0, 0x24, 0x06, 3];
    xu.extend_from_slice(&CAMERA_XU_GUID);
    xu.extend_from_slice(&[24, 1, 2, 2, 0xff, 0xff, 0]);
    xu[0] = xu.len() as u8;
    vc_body.extend_from_slice(&xu);
    // Output terminal 4 (USB streaming) fed by the extension unit.
    vc_body.extend_from_slice(&[9, 0x24, 0x03, 4, 0x01, 0x01, 0, 3, 0]);
    let total = (13 + vc_body.len()) as u16;
    let mut vc = vec![13, 0x24, 0x01, 0x10, 0x01];
    vc.extend_from_slice(&total.to_le_bytes());
    vc.extend_from_slice(&15_000_000_u32.to_le_bytes());
    vc.extend_from_slice(&[1, 1]);
    vc.extend_from_slice(&vc_body);

    let frame = |index: u8, width: u16, height: u16, intervals: &[u32]| {
        let mut f = vec![0, 0x24, 0x07, index, 0];
        f.extend_from_slice(&width.to_le_bytes());
        f.extend_from_slice(&height.to_le_bytes());
        let size = u32::from(width) * u32::from(height) * 2;
        f.extend_from_slice(&(size * 8).to_le_bytes());
        f.extend_from_slice(&(size * 8 * 120).to_le_bytes());
        f.extend_from_slice(&size.to_le_bytes());
        f.extend_from_slice(&intervals[0].to_le_bytes());
        f.push(intervals.len() as u8);
        for interval in intervals {
            f.extend_from_slice(&interval.to_le_bytes());
        }
        f[0] = f.len() as u8;
        f
    };
    let mut formats = vec![11, 0x24, 0x06, 1, 2, 0x01, 1, 0, 0, 0, 0];
    formats.extend(frame(1, 640, 480, &[83_263, 166_666, 333_333]));
    formats.extend(frame(2, 1280, 720, &[333_333]));
    formats.extend_from_slice(&[6, 0x24, 0x0d, 1, 1, 4]);
    let header_total = (14 + formats.len()) as u16;
    let mut vs = vec![14, 0x24, 0x01, 1];
    vs.extend_from_slice(&header_total.to_le_bytes());
    vs.extend_from_slice(&[0x81, 0, 4, 0, 0, 0, 1, 0]);
    vs.extend_from_slice(&formats);

    let mut body = vec![8, 0x0b, 0, 2, 0x0e, 0x03, 0, 0];
    body.extend(interface(0, 0, 1, [0x0e, 0x01, 0]));
    body.extend(vc);
    body.extend(endpoint(0x83, 0x03, 16, 6));
    body.extend_from_slice(&[5, 0x25, 0x03, 16, 0]);
    body.extend(interface(1, 0, 0, [0x0e, 0x02, 0]));
    body.extend(vs);
    for (alternate, size) in [(1, 0x0080), (2, 0x0400), (3, 0x1400)] {
        body.extend(interface(1, alternate, 1, [0x0e, 0x02, 0]));
        body.extend(endpoint(0x81, 0x05, size, 1));
    }
    [
        device(0x32e4, 0x9230, [0xef, 0x02, 0x01]),
        configuration(2, &body),
    ]
    .concat()
}

/// The ESP32-C6 USB Serial/JTAG unit (303a:1001): CDC-ACM on interfaces 0-1, vendor JTAG on 2.
pub fn esp32c6() -> Vec<u8> {
    let mut body = vec![8, 0x0b, 0, 2, 0x02, 0x02, 0, 0];
    body.extend(interface(0, 0, 1, [0x02, 0x02, 0]));
    body.extend_from_slice(&[5, 0x24, 0x00, 0x10, 0x01]);
    body.extend_from_slice(&[4, 0x24, 0x02, 0x02]);
    body.extend_from_slice(&[5, 0x24, 0x06, 0, 1]);
    body.extend_from_slice(&[5, 0x24, 0x01, 0, 1]);
    body.extend(endpoint(0x82, 0x03, 64, 1));
    body.extend(interface(1, 0, 2, [0x0a, 0, 0]));
    body.extend(endpoint(0x01, 0x02, 64, 0));
    body.extend(endpoint(0x81, 0x02, 64, 0));
    body.extend(interface(2, 0, 2, [0xff, 0xff, 0x01]));
    body.extend(endpoint(0x02, 0x02, 64, 0));
    body.extend(endpoint(0x83, 0x02, 64, 0));
    [
        device(0x303a, 0x1001, [0xef, 0x02, 0x01]),
        configuration(3, &body),
    ]
    .concat()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cdc::AcmInterfaces;
    use crate::usb::descriptor;
    use crate::uvc::descriptors::{FormatKind, UvcCamera};

    #[test]
    fn the_synthetic_camera_parses_into_the_mode_and_bandwidth_the_rig_needs() {
        let parsed = descriptor::parse(&yubi_camera()).unwrap();
        assert_eq!(
            (parsed.device.vendor_id, parsed.device.product_id),
            (0x32e4, 0x9230)
        );
        let camera = UvcCamera::parse(parsed.active().unwrap()).unwrap();
        assert_eq!(camera.uvc_version, 0x0110);
        assert_eq!(camera.clock_frequency_hz, 15_000_000);
        assert_eq!(
            (camera.camera_terminal, camera.processing_unit),
            (Some(1), Some(2))
        );
        assert_eq!(camera.extension_unit(&CAMERA_XU_GUID), Some(3));
        assert_eq!(
            (camera.control_interface, camera.streaming_interface),
            (0, 1)
        );
        assert_eq!(
            camera.mode(FormatKind::Mjpeg, 640, 480, 83_263),
            Some((1, 1, 83_263))
        );
        assert_eq!(
            camera.mode(FormatKind::Mjpeg, 640, 480, 80_000),
            Some((1, 1, 83_263))
        );
        assert_eq!(
            camera.mode(FormatKind::Uncompressed, 640, 480, 83_263),
            None
        );
        assert_eq!(camera.iso_alts.len(), 3);
        assert_eq!(camera.alt_for(3_000).map(|a| a.alternate), Some(3));
        assert_eq!(camera.alt_for(1_000).map(|a| a.alternate), Some(2));
        assert_eq!(camera.alt_for(3_073), None);
    }

    #[test]
    fn the_synthetic_encoder_has_its_cdc_acm_pair_and_no_camera() {
        let parsed = descriptor::parse(&esp32c6()).unwrap();
        let acm = AcmInterfaces::find(parsed.active().unwrap()).unwrap();
        assert_eq!((acm.communication, acm.data, acm.bulk_in), (0, 1, 0x81));
        assert!(UvcCamera::parse(parsed.active().unwrap()).is_err());
    }

    /// A descriptor set on a plain file stands in for a usbfs fd (both answer pread at
    /// offset 0): attach classifies by model *and* interfaces, and refuses an fd that is not
    /// the device Android said it was.
    #[test]
    #[cfg(target_os = "linux")]
    fn attach_classifies_from_the_fd_descriptors() {
        use crate::source::{Kind, Port};
        use std::os::fd::AsRawFd;
        let directory = tempfile::tempdir().unwrap();
        for (bytes, model, kind) in [
            (yubi_camera(), (0x32e4, 0x9230), Kind::Camera),
            (esp32c6(), (0x303a, 0x1001), Kind::Encoder),
        ] {
            let path = directory.path().join("node");
            std::fs::write(&path, &bytes).unwrap();
            let file = std::fs::File::open(&path).unwrap();
            let port =
                crate::hw::UsbfsPort::attach(file.as_raw_fd(), model.0, model.1, || Ok(0)).unwrap();
            assert_eq!(port.kind(), kind);
            assert!(
                crate::hw::UsbfsPort::attach(file.as_raw_fd(), 0x1234, model.1, || Ok(0))
                    .err()
                    .unwrap()
                    .contains("not the 1234")
            );
            // A camera model without the Sonix unit is somebody else's camera.
            if kind == Kind::Camera {
                let mut other = bytes.clone();
                let at = other.windows(16).position(|w| w == CAMERA_XU_GUID).unwrap();
                other[at] ^= 0xff;
                std::fs::write(&path, &other).unwrap();
                let file = std::fs::File::open(&path).unwrap();
                let port =
                    crate::hw::UsbfsPort::attach(file.as_raw_fd(), model.0, model.1, || Ok(0))
                        .unwrap();
                assert_eq!(port.kind(), Kind::Other);
            }
        }
    }
}
