//! ZLG USBCANFD adapters' channels, opened in userspace and commissioned as
//! `<USB serial>:<channel>`.
//!
//! One adapter carries several CAN channels under one USB serial number, so the serial alone
//! cannot say which bus a port is; the channel index (0-based, so the adapter's second
//! channel, Linux's `can1` under ZLG's kernel module, is `:1`) completes the identity, e.g.
//! `A30F115DD0120A949B80:1`. The adapter is found by enumeration and opened through
//! `sx_zlg_usbcanfd`, which claims its USB interface (refused while ZLG's Linux kernel module or
//! another process holds it); the channel opens as classic CAN at 1 Mbit/s with the adapter's
//! termination at its default, on. Two identities on one serial share the opened adapter.

use sx_zlg_usbcanfd::{ChannelOptions, Found, UsbCanFd, ZlgError};

use crate::{OpenError, Port};

/// The attached ZLG adapters, from enumeration (or a fixture).
pub struct ZlgPorts {
    attached: Box<dyn Fn() -> Result<Vec<Found>, String> + Send>,
}

/// `serial:channel`, when `identity` has that form.
#[must_use]
pub fn parse(identity: &str) -> Option<(&str, u8)> {
    let (serial, channel) = identity.rsplit_once(':')?;
    if serial.is_empty() || channel.is_empty() || !channel.bytes().all(|byte| byte.is_ascii_digit())
    {
        return None;
    }
    Some((serial, channel.parse().ok()?))
}

impl ZlgPorts {
    /// This host's adapters.
    #[must_use]
    pub fn system() -> Self {
        Self::with(|| sx_zlg_usbcanfd::list().map_err(|error| error.to_string()))
    }

    /// Adapters as `attached` reports them.
    #[must_use]
    pub fn with(attached: impl Fn() -> Result<Vec<Found>, String> + Send + 'static) -> Self {
        Self {
            attached: Box::new(attached),
        }
    }

    /// The adapter serial and channel `identity` names, when exactly one attached adapter
    /// reports that serial.
    ///
    /// # Errors
    ///
    /// [`OpenError::NotAttached`] for an identity not of the form `serial:channel` or a serial
    /// no attached adapter reports, [`OpenError::Ambiguous`] when two report it, and
    /// [`OpenError::Port`] when enumeration fails.
    pub fn channel<'a>(&self, identity: &'a str) -> Result<(&'a str, u8), OpenError> {
        let Some((serial, channel)) = parse(identity) else {
            return Err(OpenError::NotAttached(identity.to_owned()));
        };
        let attached = (self.attached)().map_err(|reason| OpenError::Port {
            serial: identity.to_owned(),
            reason,
        })?;
        let matching = attached
            .iter()
            .filter(|found| found.serial.as_deref() == Some(serial))
            .count();
        match matching {
            0 => Err(OpenError::NotAttached(identity.to_owned())),
            1 => Ok((serial, channel)),
            count => Err(OpenError::Ambiguous {
                identity: identity.to_owned(),
                bridges: (0..count).map(|_| format!("ZLG {serial}")).collect(),
            }),
        }
    }
}

/// Open channel `channel` of the adapter with USB serial `serial`, classic CAN at 1 Mbit/s.
///
/// # Errors
///
/// [`OpenError::Busy`] when another process holds the adapter's interface, naming it where the
/// host tells; any other reason the adapter or channel did not open as [`OpenError::Port`].
pub fn open(identity: &str, serial: &str, channel: u8) -> Result<Port, OpenError> {
    let port = |reason: String| OpenError::Port {
        serial: identity.to_owned(),
        reason,
    };
    let adapter = UsbCanFd::open(Some(serial)).map_err(|error| match error {
        ZlgError::Busy { holder, .. } => OpenError::Busy {
            serial: identity.to_owned(),
            device: format!("ZLG adapter {serial}"),
            holder,
        },
        error => port(error.to_string()),
    })?;
    let channel = adapter
        .open_channel(channel, ChannelOptions::default())
        .map_err(|error| port(error.to_string()))?;
    Ok(Box::new(channel))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn found(serial: &str) -> Found {
        Found {
            serial: Some(serial.to_owned()),
            vendor_id: 0x3068,
            product_id: 0x0009,
            speed: None,
        }
    }

    #[test]
    fn a_channel_is_named_by_its_adapters_serial_and_its_index() {
        assert_eq!(
            parse("A30F115DD0120A949B80:1"),
            Some(("A30F115DD0120A949B80", 1))
        );
        for bad in ["A30F", "A30F:", ":1", "A30F:x", "A30F:-1", "A30F:300"] {
            assert_eq!(parse(bad), None, "{bad}");
        }
        let ports = ZlgPorts::with(|| Ok(vec![found("A30F"), found("TWIN"), found("TWIN")]));
        assert_eq!(ports.channel("A30F:1").expect("attached"), ("A30F", 1));
        assert!(matches!(
            ports.channel("A30F"),
            Err(OpenError::NotAttached(_))
        ));
        assert!(matches!(
            ports.channel("B000:0"),
            Err(OpenError::NotAttached(_))
        ));
        assert!(matches!(
            ports.channel("TWIN:0"),
            Err(OpenError::Ambiguous { bridges, .. }) if bridges.len() == 2
        ));
        let failing = ZlgPorts::with(|| Err("enumeration failed".to_owned()));
        assert!(matches!(
            failing.channel("A30F:1"),
            Err(OpenError::Port { .. })
        ));
    }
}
