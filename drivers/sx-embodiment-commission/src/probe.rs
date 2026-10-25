//! Every openable bus scanned by its vendor crate's read-only scan.

use std::ops::RangeInclusive;

use sx_damiao_can::scan as damiao;
use sx_feetech_serial::scan::{self as feetech, STS3215_BAUD};

use crate::bridges::{Access, Bridge, enumerate};
use crate::host::Host;

/// Which ids each scan asks.
#[derive(Clone, Debug)]
pub struct Ids {
    pub feetech: Vec<u8>,
    pub damiao: Vec<u16>,
}

impl Default for Ids {
    /// Feetech ids 1 to 20 and Damiao ids 1 to `0x20`: every id a body this registry carries
    /// binds, at about 20 ms (Feetech) and 50 ms (Damiao) per silent id.
    fn default() -> Self {
        Self {
            feetech: RangeInclusive::new(1, 20).collect(),
            damiao: RangeInclusive::new(1, 0x20).collect(),
        }
    }
}

/// One serial bridge's Feetech scan.
#[derive(Clone, Debug)]
pub struct FeetechBus {
    /// The bridge's USB serial number.
    pub identity: String,
    pub tty: String,
    pub scan: Result<feetech::BusScan, String>,
}

/// One CAN interface's Damiao scan.
#[derive(Clone, Debug)]
pub struct DamiaoBus {
    /// The bridge's USB serial number, or the interface's name for a board controller.
    pub identity: String,
    pub interface: String,
    pub scan: Result<damiao::BusScan, String>,
}

/// A host's bridges and what answered on each.
#[derive(Clone, Debug, Default)]
pub struct Probe {
    pub bridges: Vec<Bridge>,
    pub feetech: Vec<FeetechBus>,
    pub damiao: Vec<DamiaoBus>,
    /// Interfaces with no USB device behind them, which carry their own name as identity.
    pub board_can: Vec<String>,
}

/// Enumerate `host` and scan each bus a scan may be sent to.
///
/// # Errors
///
/// Returns the host's failure to enumerate; a bus that fails to open or scan is recorded in
/// its entry instead.
pub fn probe<H: Host>(host: &mut H, ids: &Ids) -> std::io::Result<Probe> {
    let bridges = enumerate(host)?;
    let mut probe = Probe::default();
    for bridge in &bridges {
        let Some(identity) = bridge.identity() else {
            continue;
        };
        match &bridge.access {
            Access::Tty { path } if bridge.carries_servo_bus() => {
                let scan = host
                    .open_serial(path, STS3215_BAUD)
                    .map(|port| feetech::scan(port, ids.feetech.iter().copied()))
                    .map_err(|error| error.to_string());
                probe.feetech.push(FeetechBus {
                    identity: identity.to_owned(),
                    tty: path.clone(),
                    scan,
                });
            }
            Access::SocketCan { interfaces } => {
                for interface in interfaces {
                    probe.damiao.push(DamiaoBus {
                        identity: identity.to_owned(),
                        interface: interface.clone(),
                        scan: scan_can(host, interface, ids),
                    });
                }
            }
            _ => {}
        }
    }
    for interface in host.can_interfaces()? {
        if interface.usb_serial.is_none() {
            probe.damiao.push(DamiaoBus {
                identity: interface.name.clone(),
                interface: interface.name.clone(),
                scan: scan_can(host, &interface.name, ids),
            });
            probe.board_can.push(interface.name);
        }
    }
    probe.bridges = bridges;
    Ok(probe)
}

fn scan_can<H: Host>(host: &mut H, interface: &str, ids: &Ids) -> Result<damiao::BusScan, String> {
    let port = host
        .open_can(interface)
        .map_err(|error| error.to_string())?;
    damiao::scan(port, ids.damiao.iter().copied()).map_err(|error| error.to_string())
}
