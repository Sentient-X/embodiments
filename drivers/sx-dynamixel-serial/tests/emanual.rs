//! The packet handler builds and reads exactly the packets of the e-manual's worked examples.
//!
//! Every byte string here is copied from ROBOTIS's e-manual, Protocol 2.0
//! (<https://github.com/ROBOTIS-GIT/emanual> at `0807b1b5f7472bbbbb2c9751f1d3406678e86d37`,
//! `docs/en/dxl/protocol2.md`, sha256
//! `023de65f58daf9d41db24a26fdab5eae36c76a872ebbf564534bfcf49e08cf59`), section "Instruction
//! Details": each example's instruction packet and the status packets its devices answer.

mod common;

use common::{FakePort, hex};
use sx_dynamixel_serial::dynamixel_sdk::{
    CommResult, GroupSyncRead, GroupSyncWrite, Protocol2PacketHandler,
};

/// Ping, example 1: ID 1, an XM430-W210 (model 1030, firmware 38).
#[test]
fn ping() {
    let port = FakePort::expecting([(
        "FF FF FD 00 01 03 00 01 19 4E",
        "FF FF FD 00 01 07 00 55 00 06 04 26 65 5D",
    )]);
    let mut ph = Protocol2PacketHandler::new(port);
    assert_eq!(ph.ping(1), (1030, CommResult::Success, 0));
}

/// Read: Present Position (132, 4 bytes) of ID 1 is 166.
#[test]
fn read() {
    let port = FakePort::expecting([(
        "FF FF FD 00 01 07 00 02 84 00 04 00 1D 15",
        "FF FF FD 00 01 08 00 55 00 A6 00 00 00 8C C0",
    )]);
    let mut ph = Protocol2PacketHandler::new(port);
    assert_eq!(ph.read4_byte_tx_rx(1, 132), (166, CommResult::Success, 0));
}

/// Write: 512 to Goal Position (116, 4 bytes) of ID 1.
#[test]
fn write() {
    let port = FakePort::expecting([(
        "FF FF FD 00 01 09 00 03 74 00 00 02 00 00 CA 89",
        "FF FF FD 00 01 04 00 55 00 A1 0C",
    )]);
    let mut ph = Protocol2PacketHandler::new(port);
    assert_eq!(ph.write4_byte_tx_rx(1, 116, 512), (CommResult::Success, 0));
}

/// Sync Read: Present Position of IDs 1 and 2, 166 and 2079.
#[test]
fn sync_read() {
    // The second device's answer follows the first's on the line.
    let port = FakePort::expecting([(
        "FF FF FD 00 FE 09 00 82 84 00 04 00 01 02 CE FA",
        "FF FF FD 00 01 08 00 55 00 A6 00 00 00 8C C0 \
         FF FF FD 00 02 08 00 55 00 1F 08 00 00 BA BE",
    )]);
    let mut ph = Protocol2PacketHandler::new(port);
    let mut reader = GroupSyncRead::new(132, 4);
    assert!(reader.add_param(1) && reader.add_param(2) && !reader.add_param(2));
    assert_eq!(reader.tx_rx_packet(&mut ph), CommResult::Success);
    assert_eq!(reader.get_data(1, 132, 4), Some(166));
    assert_eq!(reader.get_data(2, 132, 4), Some(2079));
    assert_eq!(reader.get_data(2, 130, 4), None);
}

/// Sync Write: 150 and 170 to Goal Position of IDs 1 and 2; the broadcast has no answer.
#[test]
fn sync_write() {
    let port = FakePort::expecting([(
        "FF FF FD 00 FE 11 00 83 74 00 04 00 01 96 00 00 00 02 AA 00 00 00 82 87",
        "",
    )]);
    let mut ph = Protocol2PacketHandler::new(port);
    let mut writer = GroupSyncWrite::new(116, 4);
    assert!(writer.add_param(1, &[0x96, 0, 0, 0]));
    assert!(writer.add_param(2, &[0xAA, 0, 0, 0]));
    assert!(!writer.add_param(2, &[0, 0, 0, 0]));
    assert!(!writer.add_param(3, &[0, 0, 0, 0, 0]));
    assert_eq!(writer.tx_packet(&mut ph), CommResult::Success);
    assert!(ph.port().expected.is_empty());
}

/// A status packet failing its CRC, and one with no bytes at all.
#[test]
fn a_corrupt_or_absent_status_is_reported() {
    let port = FakePort::expecting([(
        "FF FF FD 00 01 03 00 01 19 4E",
        "FF FF FD 00 01 07 00 55 00 06 04 26 65 5E",
    )]);
    let mut ph = Protocol2PacketHandler::new(port);
    assert_eq!(ph.ping(1).1, CommResult::RxCorrupt);
    let port = FakePort::expecting([("FF FF FD 00 01 03 00 01 19 4E", "")]);
    let mut ph = Protocol2PacketHandler::new(port);
    assert_eq!(ph.ping(1).1, CommResult::RxTimeout);
    // Noise before the header is skipped, as the vendor skips it.
    let port = FakePort::expecting([(
        "FF FF FD 00 01 03 00 01 19 4E",
        "00 FF 12 FF FF FD 00 01 07 00 55 00 06 04 26 65 5D",
    )]);
    let mut ph = Protocol2PacketHandler::new(port);
    assert_eq!(ph.ping(1), (1030, CommResult::Success, 0));
    assert_eq!(
        hex(&ph.port().written[0]),
        "fffffd0001030001194e",
        "the request is the e-manual's"
    );
}
