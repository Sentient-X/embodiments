# sx-embodiments

Shared, dependency-free records for identifying a robot embodiment and the immutable assets that
describe it. The package owns portable facts, not runtime behavior.

An `EmbodimentManifest` can reference URDF, MJCF, USD, meshes, calibration bundles, and related
assets by URI and SHA-256 digest. Consumers remain responsible for fetching assets and for their
own adapters: forward kinematics, simulator loading, validation policy, and Rerun visualization.

The digest is mandatory. A mutable URL is a location, not an asset identity; catalog registration
must hash bytes before producing an `AssetRef`.
