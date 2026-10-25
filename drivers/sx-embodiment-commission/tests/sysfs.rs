//! The kernel's CAN interfaces read from a `sysfs` tree laid out as Linux lays it out.
#![cfg(unix)]

use std::fs;
use std::os::unix::fs::symlink;
use std::path::PathBuf;

use sx_embodiment_commission::host::CanInterface;
use sx_embodiment_commission::sysfs::{by_id, can_interfaces};

fn root(name: &str) -> PathBuf {
    let root = std::env::temp_dir().join(format!("sx-commission-{name}-{}", std::process::id()));
    let _ = fs::remove_dir_all(&root);
    fs::create_dir_all(&root).unwrap();
    root
}

#[test]
fn a_usb_adapters_channels_carry_its_serial_and_a_board_controller_none() {
    let root = root("net");
    // /sys/bus/usb/devices/1-2 is the ZLG; its interface 1-2:1.0 carries can0 and can1.
    let usb = root.join("devices/1-2");
    fs::create_dir_all(usb.join("1-2:1.0")).unwrap();
    fs::write(usb.join("serial"), "A30F115DD0120A949B80\n").unwrap();
    let net = root.join("net");
    for name in ["can1", "can0"] {
        fs::create_dir_all(net.join(name)).unwrap();
        fs::write(net.join(name).join("type"), "280\n").unwrap();
        symlink(usb.join("1-2:1.0"), net.join(name).join("device")).unwrap();
    }
    // A board controller has no USB device; an Ethernet link is not CAN.
    fs::create_dir_all(net.join("can2")).unwrap();
    fs::write(net.join("can2/type"), "280\n").unwrap();
    fs::create_dir_all(net.join("eth0")).unwrap();
    fs::write(net.join("eth0/type"), "1\n").unwrap();

    let serial = Some("A30F115DD0120A949B80".to_owned());
    assert_eq!(
        can_interfaces(&net).unwrap(),
        vec![
            CanInterface {
                name: "can0".to_owned(),
                usb_serial: serial.clone(),
            },
            CanInterface {
                name: "can1".to_owned(),
                usb_serial: serial,
            },
            CanInterface {
                name: "can2".to_owned(),
                usb_serial: None,
            },
        ]
    );
    assert!(can_interfaces(&root.join("absent")).unwrap().is_empty());
    fs::remove_dir_all(root).unwrap();
}

#[test]
fn serial_aliases_resolve_to_their_nodes() {
    let root = root("by-id");
    let node = root.join("ttyACM0");
    fs::write(&node, "").unwrap();
    let aliases = root.join("by-id");
    fs::create_dir_all(&aliases).unwrap();
    let alias = aliases.join("usb-1a86_USB_Single_Serial_5A7C118709-if00");
    symlink(&node, &alias).unwrap();
    let found = by_id(&aliases);
    let node = fs::canonicalize(node).unwrap();
    assert_eq!(
        found.get(node.to_str().unwrap()),
        Some(&alias.to_string_lossy().into_owned())
    );
    fs::remove_dir_all(root).unwrap();
}
