# sx-uvc-stereo

Side-by-side USB Video Class stereo cameras as `sx_embodiment_drivers::FrameSource`s. A ZED
delivers both eyes, exposed together, as one YUYV frame twice an eye's width; this crate captures
the one stereo track a robot's media plan names for it and hands the publisher the two planned
views.

| Product | Registry id | USB id | Eye modes (side by side) |
|---|---|---|---|
| Stereolabs ZED 2i | `zed_2i` | `2b03:f880` (video; `f881` is its sensor module, read by `sx-stereolabs-sensors`) | 2208x1242 @ 15, 1920x1080 @ 30/15, 1280x720 @ 60/30/15, 672x376 @ 100/60/30/15 ([Stereolabs video modes](https://docs.stereolabs.com/docs/video/camera-controls)); USB 2 carries VGA alone |

## Use

```rust
let plan = MediaPlan::parse(&document, &instance_id)?;
let request = CaptureRequest::for_track(&plan.tracks[0])?; // the head: left then right eye
let mut source = sx_uvc_stereo::capture::open(request)?;  // feature `capture`
let exposure = source.next()?;                            // views: [left, right], I420
```

`CaptureRequest::for_track` takes a track of exactly two views of one listed product, the units
the instance placed in them spelled `<serial>/left` and `<serial>/right` of one device, at an eye
raster and integral rate the product lists. Anything else is a typed `RequestError`.

`capture::open` finds the device by its product's USB vendor and product ids and, on Linux, by the
serial `sysfs` reports (the capture node only, `index` 0). A ZED 2i's video interface reports the
placeholder serial `OV0001` on Linux; the unit's serial is its sensor module's (`2b03:f881`), which
sits on the USB 2 half of the same connector, so the device takes that one (seen on a DGX Spark,
2026-10-26: `6-1` video, `5-1.2` sensor, serial 30371653); macOS's AVFoundation names no serial,
so there the one attached device of the product is opened and two are refused as ambiguous. It
asks for exactly the side-by-side raster, YUYV, at the planned rate, and refuses a device that
opens anything else. The camera is read on its own thread (a `nokhwa` camera may not cross
threads) into a two-frame queue. On Linux the read is V4L2's own, through the `v4l` crate, and
waits at most `STALL` (1 s): a device that stops sending frames ends the capture with an error,
its stream stopped and the device closed, instead of a read that never returns keeping it open
(on 10-27 that left a stalled ZED 2i busy for every later teleoperation interval). On Linux each
stream starts on a device just reset on its USB port (`USBDEVFS_RESET` through
`sx-capture-usb`'s usbfs, then the capture node found again, at most `RESET_WAIT`): a ZED 2i
streams once per enumeration, and on the DGX Spark (10-28) a second stream after a clean
`VIDIOC_STREAMOFF` sent no frame, and stopping it hung the GB10 xHCI controller until a reboot.
The process needs write access to the camera's `/dev/bus/usb/BBB/DDD` (the station's host-apply
grants it); without it the reset is reported and the device opened as it is. A process without camera permission (macOS) gets
`OpenError::Permission` instead of a hang.

## What it does not do

* **Exposure time.** A frame carries no device timestamp yet, so a frame is stamped when the host has it,
  later than the exposure by its readout and transfer. V4L2's own buffer timestamps would close
  that on Linux.
* **Depth, rectification.** The ZED SDK is not used; the IMU is `sx-stereolabs-sensors`. A unit's intrinsics and its eyes' relative
  pose are its factory calibration, which Stereolabs publishes per serial
  (`https://calib.stereolabs.com/?SN=<serial>`) and the registry reads
  (`sx_embodiments.factory_intrinsics.zed_factory_calibration`); `tests/fixtures/zed-2i-30371653.conf`
  is the static upper body's unit's record.

## Tests

```bash
cargo test -p sx-uvc-stereo                      # request, split, source; no hardware
cargo test -p sx-uvc-stereo --features capture   # also the device lookup
SX_UVC_STEREO_SERIAL=30371653 SX_UVC_STEREO_MODE=672x376@15 SX_UVC_STEREO_SAVE=/tmp \
  cargo test -p sx-uvc-stereo --features capture --test hardware -- --ignored --nocapture
```

`tests/fixtures/media-plans.json` is rendered by `tools/render_driver_bindings.py`: the static
upper body's head placed by serial, its eyes calibrated from the fixture record.
