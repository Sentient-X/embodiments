# AgileX Piper — official description closure

`piper_description.urdf` is the verbatim upstream file pinned by `known/piper.py`
(`PIPER_URDF`: repository `agilexrobotics/piper_ros`, revision
`ac41fcbcdda598f01b51cf6175ed9a24d0dacadc`, MIT). It references its meshes by ROS
`package://piper_description/...` URIs, so its mesh closure ships here under a directory
named after that ROS package — `piper_description/meshes/*.STL`, the same files at the
same upstream revision (`src/piper_description/meshes/`). Consumers that need the URIs to
resolve point the loader's package search path at this URDF's parent directory
(`sx_telemetry.scene` does exactly that). The closure follows the registry convention:
meshes ship on disk beside their description asset and are pinned by provenance, not
registered as embodiment assets — registering them would change the Piper embodiment digest and
change every recorded embodiment content ID.
