# PICO acquisition

Unity package `io.sentientx.embodiments.pico`, assembly `SentientEmbodiments.Pico`,
requires the consumer's pinned ByteDance PICO XR SDK 6.1.1 and Unity XR Management.
The consuming manifest resolves the SDK locally; this package does not vendor it.
`ENABLE_PICO_XR_SDK` gates compilation just as in the original capture package.

`PicoCameraCapture` owns the stereo camera handles, bounded initialization, capability
and calibration checks, RGBA buffers, acquisition clock evidence and asynchronous
cleanup. The vendor-neutral tracking reader is now
[`UnityXrTrackingSource`](../unity/README.md), shared by both headsets with unchanged
observation semantics. The camera source and `.meta` files moved byte-for-byte from SX `77f779f6355b5ee3b22395cc0fc81e5e6b3b22be`;
see [source moves](../source-moves.json). Namespaces remain `SentientCapture.Pico`.
There is no new redistribution license grant in this move.

Recording admission, controller input, status/UI, external rig coordination,
MediaCodec/file output and publication remain in the app's `io.sentientx.capture.pico`
package. A caller must await `CloseAsync` before releasing camera ownership.
Build and buffer checks are owned by the consuming capture app; physical PICO
qualification remains separate.
