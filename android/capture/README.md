# Android hardware acquisition

Canonical Kotlin sources for Quest Camera2 discovery, metadata, permissions and
sessions, and the Android USB rig bridge. Java/JNI names remain
`com.samusynth.questcamera.*`. `ISurfaceProvider` delivers camera exposure provenance
to app-owned encoders without depending on them.

The SX capture app's `android/capture/build.gradle.kts` includes this source tree
and its USB permission test through `sourceSets`, producing the same single AAR.
There is deliberately no duplicate Gradle module or second AAR. The app retains the
permission activity manifest, runtime dependencies and JNI keep rules. Encoders,
fragmented MP4, sidecars, QR/preview, pod identity, asset installation and device
reports remain in the app. Hardware sources must not import those app-owned types.

The Camera2/helper/JSON/USB implementation comes from
`Sentient-X/QuestCameraLib` at `51a486838eed8f0bdc54c75d7a1d276c4d374890`;
its parent is `samuelm2/QuestCameraLib`, original source `t-34400/QuestCameraLib`.
Pinned source history credits t-34400, samusynth, Prince and Prince baghel.
The USB CAMERA permission policy and its test were added in SX. Exact move hashes
and SX revision are in [source moves](../../xr/source-moves.json); original vendor
blobs and adaptation hashes are in the capture app's `sources.lock.json`.

QuestCameraLib had no standalone license in the inspected pinned history. This
internal extraction asserts no new license grant or public redistribution clearance;
it does not relicense the code under this repository's general license.

Run the consuming app's release unit tests, AAR build and lint after integration.
Host tests do not establish USB permission behavior or camera timing on a headset.
