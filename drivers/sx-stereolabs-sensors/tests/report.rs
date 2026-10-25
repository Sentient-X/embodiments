//! Sensor reports against byte fixtures laid out as zed-open-capture's packed `RawData`.

use std::time::Duration;

use sx_stereolabs_sensors::report::{
    ACC_SCALE, FW_3_9, REPORT_LEN, RawReport, ReportError, TEMP_NOT_VALID, ping, stream,
};

/// One `RawData` report, field by field in the source's order, little-endian.
#[allow(clippy::too_many_arguments)]
fn report(
    imu_not_valid: u8,
    timestamp: u64,
    gyro: [i16; 3],
    accel: [i16; 3],
    imu_temp: i16,
    mag_valid: u8,
    mag: [i16; 3],
    env_valid: u8,
    press: u32,
) -> Vec<u8> {
    let mut bytes = vec![0x01, imu_not_valid];
    bytes.extend(timestamp.to_le_bytes());
    for value in gyro.into_iter().chain(accel) {
        bytes.extend(value.to_le_bytes());
    }
    bytes.extend([1, 1]); // frame_sync, sync_capabilities
    bytes.extend(7_u32.to_le_bytes()); // frame_sync_count
    bytes.extend(imu_temp.to_le_bytes());
    bytes.push(mag_valid);
    for value in mag {
        bytes.extend(value.to_le_bytes());
    }
    bytes.push(0); // camera_moving
    bytes.extend(0_u32.to_le_bytes());
    bytes.push(0); // camera_falling
    bytes.extend(0_u32.to_le_bytes());
    bytes.push(env_valid);
    bytes.extend(2_512_i16.to_le_bytes()); // temp
    bytes.extend(press.to_le_bytes());
    bytes.extend(4_000_u32.to_le_bytes()); // humid
    bytes.extend(3_000_i16.to_le_bytes());
    bytes.extend(TEMP_NOT_VALID.to_le_bytes());
    assert_eq!(bytes.len(), REPORT_LEN);
    bytes.extend([0, 0]); // hid_read fills a 64-byte buffer
    bytes
}

#[test]
fn a_report_is_the_packed_layout_field_for_field() {
    let bytes = report(
        0,
        1_000,
        [100, -200, 300],
        [-4096, 0, 4096],
        3_701,
        2,
        [16, -32, 48],
        2,
        8_976_030,
    );
    let parsed = RawReport::parse(&bytes).expect("report");
    assert_eq!(parsed.timestamp, 1_000);
    assert_eq!(parsed.gyro, [100, -200, 300]);
    assert_eq!(parsed.accel, [-4096, 0, 4096]);
    assert_eq!(parsed.frame_sync_count, 7);
    assert_eq!(parsed.mag, [16, -32, 48]);
    assert_eq!(parsed.press, 8_976_030);
    assert_eq!(parsed.humid, 4_000);
    assert_eq!(parsed.temp_cam_right, TEMP_NOT_VALID);
}

#[test]
fn counts_become_rep_103_units_at_the_sources_scales() {
    let bytes = report(
        0,
        1_000,
        [32_767, 0, 0],
        [4096, 0, 0],
        3_701,
        2,
        [16, 0, 0],
        2,
        8_976_030,
    );
    let parsed = RawReport::parse(&bytes).expect("report");
    // 4096 counts at +-8 g full scale is one g of the source's DEFAULT_GRAVITY.
    assert!((parsed.acceleration_mps2()[0] - 9.8189).abs() < 1e-12);
    assert!((ACC_SCALE * 32_768.0 / 8.0 - 9.8189).abs() < 1e-12);
    // 32767 counts is just under 1000 deg/s.
    let expected = (32_767.0_f64 * 1000.0 / 32_768.0).to_radians();
    assert!((parsed.angular_velocity_radps()[0] - expected).abs() < 1e-12);
    // 16 counts is one microtesla.
    assert!((parsed.magnetic_field_t().expect("new")[0] - 1e-6).abs() < 1e-18);
    // Firmware 3.9 and later: 1e-4 hPa a count; earlier: 1e-2 hPa.
    assert!((parsed.pressure_pa(FW_3_9).expect("new") - 89_760.3).abs() < 1e-6);
    assert!((parsed.pressure_pa(FW_3_9 - 1).expect("new") - 8_976_030.0).abs() < 1e-3);
    assert!((parsed.imu_temperature_c().expect("measured") - 37.01).abs() < 1e-12);
    // A tick is 39.0625 us.
    assert_eq!(parsed.device_time(), Duration::from_nanos(39_062_500));
}

#[test]
fn stale_or_absent_readings_are_not_reported_new() {
    let bytes = report(1, 0, [0; 3], [0; 3], TEMP_NOT_VALID, 1, [0; 3], 0, 0);
    let parsed = RawReport::parse(&bytes).expect("report");
    assert!(!parsed.imu_is_new());
    assert!(parsed.magnetic_field_t().is_none());
    assert!(parsed.pressure_pa(FW_3_9).is_none());
    assert!(parsed.imu_temperature_c().is_none());
}

#[test]
fn a_short_report_or_another_id_is_refused() {
    let bytes = report(0, 0, [0; 3], [0; 3], 0, 0, [0; 3], 0, 0);
    assert_eq!(RawReport::parse(&bytes[..40]), Err(ReportError::Short(40)));
    let mut other = bytes.clone();
    other[0] = 0x32;
    assert_eq!(
        RawReport::parse(&other),
        Err(ReportError::NotSensorData(0x32))
    );
}

#[test]
fn feature_reports_are_the_sources_requests() {
    assert_eq!(stream(true), [0x32, 1]);
    assert_eq!(stream(false), [0x32, 0]);
    assert_eq!(ping(), [0x21, 0xF2]);
}
