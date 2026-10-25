//! The Linux kernel's view of CAN interfaces and serial aliases, read from `sysfs` and `/dev`.
//!
//! A CAN interface under `/sys/class/net` has link type 280 (`ARPHRD_CAN`); its `device` link
//! names the USB interface it sits in, whose parent USB device carries the `serial`. This is
//! the identity `sx-embodiment-bodies`' `socketcan` module opens an interface by. Off Linux the
//! directories do not exist and both lists are empty.

use std::collections::BTreeMap;
use std::fs;
use std::io;
use std::path::Path;

use crate::host::CanInterface;

/// The kernel's link type for a CAN interface (`ARPHRD_CAN`).
const CAN_LINK_TYPE: &str = "280";

/// Every CAN interface under `net` (`/sys/class/net`), sorted by name.
///
/// # Errors
///
/// Returns a failure to read a directory that exists.
pub fn can_interfaces(net: &Path) -> io::Result<Vec<CanInterface>> {
    let entries = match fs::read_dir(net) {
        Ok(entries) => entries,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(error) => return Err(error),
    };
    let mut found = Vec::new();
    for entry in entries {
        let path = entry?.path();
        if !fs::read_to_string(path.join("type")).is_ok_and(|kind| kind.trim() == CAN_LINK_TYPE) {
            continue;
        }
        let Some(name) = path
            .file_name()
            .map(|name| name.to_string_lossy().into_owned())
        else {
            continue;
        };
        found.push(CanInterface {
            name,
            usb_serial: usb_serial(&path),
        });
    }
    found.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(found)
}

fn usb_serial(interface: &Path) -> Option<String> {
    let device = fs::canonicalize(interface.join("device")).ok()?;
    let serial = fs::read_to_string(device.parent()?.join("serial")).ok()?;
    Some(serial.trim().to_owned()).filter(|serial| !serial.is_empty())
}

/// Each device node's stable alias under `by_id` (`/dev/serial/by-id`), keyed by the node it
/// resolves to.
#[must_use]
pub fn by_id(by_id: &Path) -> BTreeMap<String, String> {
    let Ok(entries) = fs::read_dir(by_id) else {
        return BTreeMap::new();
    };
    entries
        .filter_map(Result::ok)
        .filter_map(|entry| {
            let target = fs::canonicalize(entry.path()).ok()?;
            Some((
                target.to_string_lossy().into_owned(),
                entry.path().to_string_lossy().into_owned(),
            ))
        })
        .collect()
}
