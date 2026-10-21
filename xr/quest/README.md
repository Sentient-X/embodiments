# Quest acquisition

Unity package `io.sentientx.embodiments.quest`, assembly `SentientEmbodiments.Quest`,
requires Meta XR Core 81.0.0. `QuestPoseSource` selects the existing immediate/raw
pose API and filters repeated native timestamps. `QuestImuSource` retains the
OpenXR missing-acceleration velocity derivative and its sampling history.
`QuestBodySource` owns the full-body buffer, native tracking start/stop and high
fidelity request. The consumer still decides when to warm the tracker, poll, reset
sampling history and end its lifetime; each source belongs to one acquisition owner.

These readers are extracted from `PoseLogger`, `IMULogger` and `BodyTrackingLogger`
in OpenQuestCapture `6efbd4bb81dad6df91bd5bda42624d6c4a88db22`, through SX
`77f779f6355b5ee3b22395cc0fc81e5e6b3b22be`. `RealityLog.OVR` and the serialized
`PoseStateMode` values remain stable. CSV formatting, timestamp-to-wall-clock
mapping, session statistics and file publication remain in those app loggers.
Display-rate policy and scene/permission orchestration remain app-owned.

The [OpenQuestCapture MIT notice](LICENSE.OpenQuestCapture) applies to the extracted
source. Original authors are t-34400 and samuelm2; subsequent upstream work is
preserved in the pinned OpenQuestCapture history and the capture app source lock.
