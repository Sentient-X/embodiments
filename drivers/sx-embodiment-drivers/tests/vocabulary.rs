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

#[test]
fn every_model_is_rated() {
    for model in ActuatorModel::ALL {
        assert!(model.rating().peak_torque_nm > 0.0, "{}", model.wire());
    }
    let j8009p = ActuatorModel::new("damiao_dm8009p").rating();
    assert_eq!(j8009p.supply_volts, Some([24.0, 48.0]));
    assert_eq!(j8009p.admits_supply(48.0), Some(true));
    assert_eq!(j8009p.admits_supply(50.4), Some(false));
    assert_eq!(
        ActuatorModel::new("damiao_dm8009")
            .rating()
            .admits_supply(24.0),
        None
    );
}
