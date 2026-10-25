//! The ports a body's rendered bindings declare, and what must answer on each.
//!
//! Read from the registry's rendered bindings (`tools/render_driver_bindings.py`): each axis's
//! joint, actuator model, bus kind and chain address, and the port its bus is commissioned
//! under (`port`, the registry's `ActuatorBinding` port). A document rendered before axes
//! carried a port puts a Damiao axis on `can`, the one port `sx-embodiment-bodies` opens a
//! single-bus Damiao body through, and any other axis on a port named for its bus kind. Other
//! members of the document (bounds, sign, reduction) are the driver's, not commissioning's,
//! and are not read.

use serde::Deserialize;
use sx_damiao_can::DAMIAO_CAN;
use sx_feetech_serial::FEETECH_SERIAL;
use thiserror::Error;

/// One actuator a port must carry.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DeclaredUnit {
    pub joint: String,
    pub bus_id: u16,
    /// The registry's actuator model (`feetech_sts3215`, `damiao_dm8009`).
    pub model: String,
}

/// The bus kinds commissioning can scan.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BusKind {
    Feetech,
    Damiao,
}

/// One port and the units on it, in native state order.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DeclaredPort {
    pub name: String,
    pub bus: BusKind,
    pub units: Vec<DeclaredUnit>,
}

/// A body's declared ports, in the order their first axis appears.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DeclaredBody {
    pub embodiment: String,
    pub id: String,
    pub ports: Vec<DeclaredPort>,
}

#[derive(Debug, Error, Eq, PartialEq)]
pub enum DeclaredError {
    #[error("the bindings document does not parse: {0}")]
    Document(String),
    #[error("joint {joint} is on bus {bus}, which commissioning cannot scan")]
    Bus { joint: String, bus: String },
    #[error("port {port} carries both {first} and {second} buses")]
    MixedPort {
        port: String,
        first: String,
        second: String,
    },
    #[error("port {port} binds address {bus_id} twice")]
    DuplicateId { port: String, bus_id: u16 },
}

#[derive(Deserialize)]
struct Document {
    embodiment: String,
    id: String,
    axes: Vec<Axis>,
}

#[derive(Deserialize)]
struct Axis {
    joint: String,
    model: String,
    bus: String,
    bus_id: u16,
    #[serde(default)]
    port: Option<String>,
}

impl DeclaredBody {
    /// Parse one or more rendered bindings documents of one body (a body whose buses are
    /// rendered apart is passed every document).
    ///
    /// # Errors
    ///
    /// Refuses an unparseable document, a bus commissioning cannot scan, a port carrying two
    /// bus kinds and an address bound twice on one port.
    pub fn parse(documents: &[&str]) -> Result<Self, DeclaredError> {
        let mut body: Option<Self> = None;
        for document in documents {
            let parsed: Document = serde_json::from_str(document)
                .map_err(|error| DeclaredError::Document(error.to_string()))?;
            let target = body.get_or_insert_with(|| Self {
                embodiment: parsed.embodiment.clone(),
                id: parsed.id.clone(),
                ports: Vec::new(),
            });
            for axis in parsed.axes {
                target.add(axis)?;
            }
        }
        body.ok_or_else(|| DeclaredError::Document("no bindings document".to_owned()))
    }

    fn add(&mut self, axis: Axis) -> Result<(), DeclaredError> {
        let bus = if axis.bus == FEETECH_SERIAL.wire() {
            BusKind::Feetech
        } else if axis.bus == DAMIAO_CAN.wire() {
            BusKind::Damiao
        } else {
            return Err(DeclaredError::Bus {
                joint: axis.joint,
                bus: axis.bus,
            });
        };
        let name = axis.port.unwrap_or_else(|| match bus {
            BusKind::Damiao => "can".to_owned(),
            BusKind::Feetech => axis.bus.clone(),
        });
        let unit = DeclaredUnit {
            joint: axis.joint,
            bus_id: axis.bus_id,
            model: axis.model,
        };
        if let Some(port) = self.ports.iter_mut().find(|port| port.name == name) {
            if port.bus != bus {
                return Err(DeclaredError::MixedPort {
                    port: name,
                    first: format!("{:?}", port.bus),
                    second: format!("{bus:?}"),
                });
            }
            if port.units.iter().any(|known| known.bus_id == unit.bus_id) {
                return Err(DeclaredError::DuplicateId {
                    port: name,
                    bus_id: unit.bus_id,
                });
            }
            port.units.push(unit);
        } else {
            self.ports.push(DeclaredPort {
                name,
                bus,
                units: vec![unit],
            });
        }
        Ok(())
    }
}
