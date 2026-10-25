# sx-stereolabs-sensors

The Stereolabs ZED 2i's sensor module as an `sx_embodiment_drivers::InertialSource`. A ZED 2i
attaches as two USB devices: the stereo camera (`2b03:f880`, read by `sx-uvc-stereo`) and its
sensor module (`2b03:f881`, "ZED-2i HID INTERFACE"), which streams one 62-byte HID report about
every 2.5 ms: accelerometer and gyroscope (400 Hz), magnetometer (about 50 Hz), barometer
(about 25 Hz) and temperatures. The sensor module reports the unit's real serial; the video
interface reports a placeholder (`OV0001`).

| Product | Registry id | USB id | Measures |
|---|---|---|---|
| Stereolabs ZED 2i | `zed_2i` | `2b03:f881` | acceleration, angular velocity, magnetic field, pressure, temperature |

## Use

```rust
let plan = MediaPlan::parse(&document, &instance_id)?;
let mut imu = sx_stereolabs_sensors::hid::open(&plan.inertial[0])?; // feature `hid`
let sample = imu.next()?; // on the module's axes, m/s², rad/s, T, Pa, °C
let up_in_mount = plan.inertial[0].in_mount(sample.acceleration_mps2);
```

`SensorRequest::for_sensor` takes a planned sensor of a listed product whose unit (the serial the
instance placed in its slot) is a Stereolabs serial; `hid::open` finds the module by its USB ids
and that serial, starts its stream, pings it every 400 reports as `zed-open-capture` does, and stops
the stream on drop. A sample is yielded for every new accelerometer and gyroscope reading; a
reading the module marks stale is skipped, and two seconds without one is a device failure.

The module's axes are Stereolabs' RAW frame (x down, y towards the left eye, z out of the lenses).
Where they sit is the registry's: `sx_embodiments.known.zed.zed_2i_imu` mounts them on the body's
`ZED_camera` frame, a quarter turn about its y axis.

## Calibration reading

```bash
cargo run -p sx-stereolabs-sensors --features hid --bin sx-imu-still -- \
  --plan plan.json --instance <instance id> --seconds 5
```

Hold the camera still. It prints the rate on both clocks, the mean specific force and gyro bias on
the sensor's axes, and the up direction, roll and pitch of the mount link (`ZED_camera`): the
camera's tilt from level. It only reads.

## Departures from zed-open-capture

`src/report.rs` lists them: REP-103 units, the device clock reported as it is (no rebasing onto
the wall clock or video-frame drift scaling), the first sample kept, humidity and imager
temperatures not converted. `LICENSE-THIRD-PARTY` carries Stereolabs' MIT notice.

## Tests

```bash
cargo test -p sx-stereolabs-sensors              # layout, scales, source, request; no hardware
cargo test -p sx-stereolabs-sensors --features hid --test hardware -- --ignored --nocapture
```
