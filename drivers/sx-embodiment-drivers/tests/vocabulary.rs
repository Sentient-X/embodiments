//! The driver crate's actuator vocabulary is the registry's, rendered by
//! `tools/render_driver_bindings.py` into `bindings/vocabulary.json`.

use serde_json::Value;
use sx_embodiment_drivers::{ActuatorBus, ActuatorModel};

#[test]
fn the_vocabulary_is_the_registrys() {
    let rendered: Value =
        serde_json::from_str(include_str!("../bindings/vocabulary.json")).expect("vocabulary");
    let wires = |key: &str| -> Vec<String> {
        rendered[key]
            .as_array()
            .expect(key)
            .iter()
            .map(|wire| wire.as_str().expect("a wire value").to_owned())
            .collect()
    };
    let models: Vec<String> = ActuatorModel::ALL
        .iter()
        .map(|model| model.wire().to_owned())
        .collect();
    let buses: Vec<String> = ActuatorBus::ALL
        .iter()
        .map(|bus| bus.wire().to_owned())
        .collect();
    assert_eq!(models, wires("models"));
    assert_eq!(buses, wires("buses"));
}
