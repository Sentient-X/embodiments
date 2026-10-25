//! The source over a scripted HID link, and the request from the rendered plan.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use sx_embodiment_drivers::{InertialSource, MediaPlan};
use sx_stereolabs_sensors::report::{PING_EVERY, REPORT_LEN};
use sx_stereolabs_sensors::{ReportPort, RequestError, SensorRequest, StereolabsInertialSource};

const PLANS: &str = include_str!("../../sx-uvc-stereo/tests/fixtures/media-plans.json");
const NOMINAL: &str = include_str!("../../sx-embodiment-drivers/tests/fixtures/media-plans.json");

fn plan(fixtures: &str, key: &str) -> MediaPlan {
    let value: serde_json::Value = serde_json::from_str(fixtures).expect("fixtures");
    let document = value[key].to_string();
    let instance = value[key]["instance"]
        .as_str()
        .expect("instance")
        .to_owned();
    MediaPlan::parse(&document, &instance).expect("plan")
}

#[derive(Default)]
struct Script {
    reports: VecDeque<Vec<u8>>,
    features: Vec<Vec<u8>>,
}

struct Port(Arc<Mutex<Script>>);

impl ReportPort for Port {
    fn read(&mut self, buffer: &mut [u8], _timeout: Duration) -> Result<usize, String> {
        let mut script = self.0.lock().expect("script");
        let Some(report) = script.reports.pop_front() else {
            return Err("unplugged".to_owned());
        };
        buffer[..report.len()].copy_from_slice(&report);
        Ok(report.len())
    }

    fn send_feature(&mut self, report: &[u8]) -> Result<(), String> {
        self.0
            .lock()
            .expect("script")
            .features
            .push(report.to_vec());
        Ok(())
    }
}

fn reading(imu_not_valid: u8, ticks: u64, accel_x: i16) -> Vec<u8> {
    let mut bytes = vec![0_u8; REPORT_LEN];
    bytes[0] = 0x01;
    bytes[1] = imu_not_valid;
    bytes[2..10].copy_from_slice(&ticks.to_le_bytes());
    bytes[16..18].copy_from_slice(&accel_x.to_le_bytes());
    bytes
}

#[test]
fn the_head_imu_is_its_zed_by_serial() {
    let plan = plan(PLANS, "static-upper-body-head");
    let request = SensorRequest::for_sensor(&plan.inertial[0]).expect("request");
    assert_eq!(request.serial, "30371653");
    assert_eq!(
        (request.product.usb_vendor, request.product.usb_product),
        (0x2b03, 0xf881)
    );
    assert_eq!(&*request.frame, "head_imu");
}

#[test]
fn a_sensor_without_a_unit_or_of_another_product_is_refused() {
    let nominal = plan(NOMINAL, "sentient-static-upper-body");
    assert!(matches!(
        SensorRequest::for_sensor(&nominal.inertial[0]),
        Err(RequestError::NoUnit { .. })
    ));
    let mut other = plan(PLANS, "static-upper-body-head").inertial[0].clone();
    other.model = "bno085".to_owned();
    assert!(matches!(
        SensorRequest::for_sensor(&other),
        Err(RequestError::Product(_))
    ));
    let mut lettered = plan(PLANS, "static-upper-body-head").inertial[0].clone();
    lettered.unit = Some("OV0001".to_owned());
    assert!(matches!(
        SensorRequest::for_sensor(&lettered),
        Err(RequestError::Serial { .. })
    ));
}

#[test]
fn each_new_reading_is_one_sample_stale_and_foreign_reports_are_skipped() {
    let script = Arc::new(Mutex::new(Script::default()));
    {
        let mut script = script.lock().expect("script");
        script.reports.extend([
            reading(0, 100, 4096),
            reading(1, 101, 0), // stale: the IMU repeats
            vec![0x32, 1],      // a stream-status echo
            reading(0, 102, -4096),
        ]);
    }
    let mut source =
        StereolabsInertialSource::start(Port(Arc::clone(&script)), Arc::from("head_imu"), 779)
            .expect("start");
    let first = source.next().expect("first");
    assert_eq!(&*first.frame, "head_imu");
    assert!((first.acceleration_mps2[0] - 9.8189).abs() < 1e-12);
    let second = source.next().expect("second");
    assert!((second.acceleration_mps2[0] + 9.8189).abs() < 1e-12);
    assert_eq!(
        second.device_time.saturating_sub(first.device_time),
        Duration::from_nanos(78_125)
    );
    assert!(source.next().is_err(), "an unplugged link ends the source");
    drop(source);
    let features = script.lock().expect("script").features.clone();
    assert_eq!(features.first(), Some(&vec![0x32, 1]));
    assert_eq!(features.last(), Some(&vec![0x32, 0]));
}

#[test]
fn the_module_is_pinged_as_the_source_pings_it() {
    let script = Arc::new(Mutex::new(Script::default()));
    script
        .lock()
        .expect("script")
        .reports
        .extend((0..u64::from(PING_EVERY)).map(|tick| reading(0, tick, 0)));
    let mut source =
        StereolabsInertialSource::start(Port(Arc::clone(&script)), Arc::from("imu"), 779)
            .expect("start");
    for _ in 0..PING_EVERY {
        source.next().expect("sample");
    }
    let pings = script
        .lock()
        .expect("script")
        .features
        .iter()
        .filter(|report| report.as_slice() == [0x21, 0xF2])
        .count();
    assert_eq!(pings, 1);
}
