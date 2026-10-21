# XR device adapters

`quest/` and `pico/` are Unity packages that read headset hardware. The capture app
references their assemblies; these packages never reference the app. Device data,
validity, timestamps and vendor errors belong here. Collection admission, sessions,
controls, companion HTTP, authentication, encoding/publication and episode identity
remain with the consumer.

- [Quest](quest/README.md): native pose/IMU samples and the full-body tracker.
- [Pico](pico/README.md): vendor stereo camera ownership and Unity XR tracking.
- [Android](../android/capture/README.md): Camera2, permission and USB device access.

[Source moves](source-moves.json) pins the exact SX revision and hashes for files
moved without modification. The superproject's
`experience/data-factory/capture-app/sources.lock.json` records upstream commit,
original blob/digest and both retained app and extracted hardware destinations.
Splits retain both destinations against the original source; this is not a wholesale
upstream checkout. The packages retain their original namespaces and existing Unity
script GUIDs. There is no blanket change of source licensing.

Build and host verification are coordinated by the consuming SX capture app. A
successful build or host harness does not qualify camera timing, tracking or USB on a
physical headset. No new physical qualification is asserted by this extraction.
