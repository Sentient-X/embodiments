# sx-capture-usb

The capture device drivers for a borrowed Android/Linux usbfs descriptor. `usb` owns
descriptor parsing and usbfs control/isochronous/bulk operations; `uvc` owns camera
probe/commit, payload reconstruction, device-clock mapping and controls. `cdc` owns the
ESP32-C6 CDC-ACM link, its single buffered reader, line coding, and DTR/RTS recovery.
`hw::UsbfsPort` binds them to YUBI device identities from `sx-yubi`.

`source` owns the driver contracts (`Port`, camera/encoder streams, source frames,
exposure control and disconnection errors). The recorder imports those types directly.
The caller supplies a monotonic-clock function at attachment and an `ExposureConfig`
at camera open. Requested controls are checked against those bounds before USB writes;
the driver reads controls back. No pod package is a dependency. Device descriptors stay
borrowed: the caller keeps the fd valid until the driver releases its interfaces.

Recording policy stays in the pod: source-rate admission, frame selection, software
exposure decisions, encoder liveness windows, receipt timestamps, CSV/media files,
sealing, authentication and custody. The driver carries no Factory or collection state.
The transported implementation retains its unsafe usbfs boundary and existing lint
level; its kernel layouts and transfer ownership tests move with the implementation.

Run `cargo test -p sx-capture-usb` from the embodiment workspace. Host tests cover
protocol bytes, bounds before I/O, stream reconstruction, and CDC recovery. Linux/Android
also compile the usbfs implementation and run its platform-specific fixtures. Host tests
and APK builds do not establish attached-device or physical qualification.
