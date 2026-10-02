//! The B601 chain binds exactly the registry's facts, rendered by
//! `tools/render_driver_bindings.py` into `fixtures/b601_bindings.json`.

// Equal means bit-for-bit equal: both sides are the same f64 literals.
#![allow(clippy::float_cmp)]

use serde_json::Value;
use sx_damiao_can::b601::b601;

#[test]
fn the_b601_chain_is_the_registrys_bindings_in_native_state_order() {
    let rendered: Value =
        serde_json::from_str(include_str!("fixtures/b601_bindings.json")).expect("bindings");
    assert_eq!(rendered["embodiment"], "b601-dm");
    let axes = rendered["axes"].as_array().unwrap();
    let chain = b601();
    assert_eq!(chain.len(), axes.len());
    for (motor, fact) in chain.iter().zip(axes) {
        let bound = &motor.axis;
        let binding = bound.binding;
        assert_eq!(bound.joint, fact["joint"].as_str().unwrap());
        assert_eq!(bound.lower, fact["lower"].as_f64().unwrap(), "{fact}");
        assert_eq!(bound.upper, fact["upper"].as_f64().unwrap(), "{fact}");
        assert_eq!(binding.model.wire(), fact["model"].as_str().unwrap());
        assert_eq!(binding.bus.wire(), fact["bus"].as_str().unwrap());
        assert_eq!(u64::from(binding.bus_id), fact["bus_id"].as_u64().unwrap());
        assert_eq!(i64::from(binding.sign), fact["sign"].as_i64().unwrap());
        assert_eq!(binding.zero_offset, fact["zero_offset"].as_f64().unwrap());
        assert_eq!(
            binding.reduction,
            fact["reduction"].as_f64().unwrap(),
            "{fact}"
        );
    }
}
