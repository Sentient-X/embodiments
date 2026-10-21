# Unity XR acquisition

`io.sentientx.embodiments.xr` / `SentientEmbodiments.XR` samples Unity XR head and
controller poses, tracking-origin changes and controller feature availability. It
has no vendor SDK, app lifecycle, network, authentication or robot target mapping.

`UnityXrTrackingSource` is extracted from `PicoTrackingCapture`; the observation
field names, values and sampling semantics are unchanged, including explicit
unavailable native pose times and body tracking. Its Unity GUID is retained.
See [source moves](../source-moves.json) for the original SX source identity and
adaptation digest. The extraction grants no new redistribution license.
`UnityXrControls` is new device-only feature reading; neutral rearming and engagement
admission belong to the consuming application/runtime. These reads are not a
qualified controller-to-robot mapping.
