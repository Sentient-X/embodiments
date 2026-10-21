# Linux capture transport

`sx-capture-linux` owns the laptop YUBI USB discovery, verified camera/encoder
opening, terminal configuration, camera XU identity requests and V4L2 transport.
It uses the same `sx-yubi` identity protocol as the Android USB driver.

A camera retains the exclusive lock and pinned connection through streaming;
controls use that same verified handle. Encoder terminal settings are changed
only after verifying the opened node against the pinned USB connection. Frame
reads require monotonic start-of-exposure timestamps; previews make no timing
claim. A stream timeout is terminal for that stream.

The pod owns calibration and role assignment, sampling and readiness deadlines,
frame selection, exposure policy, worker scheduling, recording, recovery and
Factory admission. Driver functions accept the pod's format, rate, control choices
and timeout. No capture worker, file writer, network client or credential lives here.

`cargo test -p sx-capture-linux` uses files and pseudo-terminals only. The
`test-support` feature exposes a fake sysfs/devfs inventory so the pod's existing
Factory binding test still crosses real discovery and role assignment without
physical devices. Linux-only transport and hotplug/locking tests require Linux;
a host build does not establish physical qualification.
