# sx-realsense

Intel RealSense D400 cameras (D405, D435, D435i) as the media plan's `FrameSource`s, through
librealsense2's own SDK ([`realsense-rust`](https://gitlab.com/tangram-vision/oss/realsense-rust)
1.3.0, Apache-2.0), which the `sdk` feature turns on. Without it the crate builds anywhere, and
every rule below is tested against recorded framesets.

Consumer: the `sx` station's media publisher, over `sdk::open`, once it opens sources from its
robot's media plan.

| Item | Path |
|---|---|
| What a track asks of a device | `CaptureRequest::for_track(&Track, Emitter)`: the device serial and stream of each view's placed unit, raster and rate |
| A camera unit | `RealSenseUnit`, spelled `<device serial>/color`, `/infrared_1` or `/infrared_2`: one unit per imager, each with its own intrinsics |
| A track's capture | `RealSenseSource<D: Device>`, a `FrameSource` over any frameset source |
| librealsense2 | `sdk::open(request)` returns the source and each view's `FactoryIntrinsics`; `sdk::devices()` lists what is attached |
| A replay | `recorded::RecordedDevice` |
| Capture time | `clock::captured_at`: a global-time exposure on the host's monotonic and wall clocks |
| Factory intrinsics | `FactoryIntrinsics::document()`, `sx.factory-intrinsics/v1`, read by `sx_embodiments.factory_intrinsics` |
| The projector | `Emitter::{On, Off, Interleaved}` |

- **One exposure.** A track is one stream, or the infrared pair as infrared 1 left and
  infrared 2 right, all of one device; colour and infrared are never one track (different
  shutters). A pair's frames must share a frame number and timestamp.
- **Capture time.** Frames must be stamped in librealsense's global-time domain (the device
  clock regressed onto the host's wall clock; the source enables it on every sensor). A paired
  host sample (monotonic midpoint around a wall read) is taken as each frameset arrives, and
  the frame's age at it is subtracted from both clocks. The error is the SDK's regression error
  plus the sample's half-width, and an epoch timestamp in f64 milliseconds resolves to a
  fraction of a microsecond. A timestamp up to 10 ms ahead of the host is the regression's
  uncertainty and is clamped; later is refused, as is any other domain (system time is host
  arrival, not exposure). This is the mapping `sx_drivers.realsense` uses.
- **Projector.** `Interleaved` runs the device at twice the planned rate with
  `RS2_OPTION_EMITTER_ON_OFF` and publishes an infrared track's dark frames only, read from
  each frame's laser-power-mode metadata; a frame without that metadata is refused.
- **Pixels.** Colour is RGB8 and infrared Y8, converted to I420 in BT.601 limited range.
  `realsense-rust` 1.3.0 picks a YUYV pixel's luma by row parity instead of column parity, so
  YUYV is never requested.
- **Factory intrinsics.** Each started profile's intrinsics, in the SDK's terms. The reader maps
  Brown-Conrady to `plumb_bob`, Kannala-Brandt to `kannala_brandt`, a coefficient-free inverse
  Brown-Conrady to `none`, and refuses the rest; the calibration is
  `CalibrationMethod.VENDOR_FACTORY` with no measurement time.

```bash
cargo test -p sx-realsense
# against librealsense2 2.x (on a Mac with Homebrew: RUSTFLAGS="-L /opt/homebrew/lib")
cargo test -p sx-realsense --features sdk
SX_REALSENSE_SERIAL=<serial> cargo test -p sx-realsense --features sdk --test hardware -- --ignored
```
