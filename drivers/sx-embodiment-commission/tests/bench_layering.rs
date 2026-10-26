//! Never skip a layer: a bench tool reaches a robot through the embodiment layer
//! (`sx_embodiment_bodies` to open or observe a body, the governor to move it), never by a
//! vendor's protocol, transport or serial port. Three such tools broke on 2026-10-26 the moment
//! the registry changed. This check reads every bench binary's source and refuses one that
//! names a vendor crate, a CAN or serial port, or a USB stack.
//!
//! A tool that still does is listed in [`NOT_YET`] with why, and the list only shrinks: a
//! listed tool that no longer names a vendor fails the check until its entry is removed.

use std::path::Path;

/// What a bench binary must not name: the vendor crates, the raw ports and the USB stacks.
const VENDOR: &[&str] = &[
    "sx_damiao_can",
    "sx_feetech_serial",
    "sx_dynamixel_serial",
    "sx_zlg_usbcanfd",
    "sx_socketcan",
    "sx_embodiment_drivers::can",
    "sx_embodiment_drivers::serial",
    "serialport",
    "nusb",
];

/// Bench binaries still to move onto the layer, and why they have not.
const NOT_YET: &[(&str, &str)] = &[(
    "sx-identify.rs",
    "opens DamiaoChain over a ZLG channel and reads registers with the Damiao scan; to move \
     onto open_with and observe, its torso and wrist skips onto out_of_service",
)];

#[test]
fn bench_tools_go_through_the_embodiment_layer() {
    let bins = Path::new(env!("CARGO_MANIFEST_DIR")).join("src/bin");
    let mut refused = Vec::new();
    let mut stale = Vec::new();
    for entry in std::fs::read_dir(&bins).expect("src/bin") {
        let path = entry.expect("an entry").path();
        let name = path
            .file_name()
            .and_then(|name| name.to_str())
            .expect("a file name")
            .to_owned();
        let source = std::fs::read_to_string(&path).expect("source");
        let named: Vec<&str> = VENDOR
            .iter()
            .copied()
            .filter(|vendor| source.contains(vendor))
            .collect();
        let listed = NOT_YET.iter().any(|(bin, _)| *bin == name);
        match (named.is_empty(), listed) {
            (false, false) => refused.push(format!("{name} names {}", named.join(", "))),
            (true, true) => stale.push(name),
            _ => {}
        }
    }
    assert!(
        refused.is_empty(),
        "bench tools skipping the embodiment layer (open or observe the body through \
         sx_embodiment_bodies instead): {refused:?}"
    );
    assert!(
        stale.is_empty(),
        "these tools go through the layer now; remove them from NOT_YET: {stale:?}"
    );
}
