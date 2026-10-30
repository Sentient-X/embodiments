//! A Damiao body on one bus, opened from data: its rendered bindings, its answer-id
//! convention and each joint's controller configuration.
//!
//! Every body whose Damiao motors share one CAN bus, each answering on its send id plus one
//! offset, is one [`DamiaoBody`] constant: the humanoid upper body
//! ([`crate::humanoid::HUMANOID_UPPER_BODY`]) and the static upper body
//! ([`crate::static_upper_body::STATIC_UPPER_BODY`]). No bound, address, model, sign or reduction
//! is spelled in Rust; the bindings are the registry's, rendered by
//! `tools/render_driver_bindings.py`. What a constant adds is controller configuration keyed
//! by joint name, refused unless its joints are the bindings' in their order. The B601-DM is
//! not one: its gripper is a second chain with its own transmission and soft box.

use std::time::Duration;

use sx_embodiment_drivers::can::CanPort;
use sx_embodiment_drivers::{BodyBindings, BodyError, BoundAxis, InstanceDrive};

use crate::chain::{DAMIAO_CAN, DamiaoAxis, DamiaoChain, DamiaoChainError};

/// One joint's MIT controller configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointGains {
    pub joint: &'static str,
    /// Position gain, 0..=500.
    pub kp: f64,
    /// Damping, 0..=5.
    pub kd: f64,
}

impl JointGains {
    #[must_use]
    pub const fn new(joint: &'static str, kp: f64, kd: f64) -> Self {
        Self { joint, kp, kd }
    }
}

/// One Damiao body on one bus.
#[derive(Clone, Copy, Debug)]
pub struct DamiaoBody {
    /// The registry name of the body whose bus this is: what the opener finds this bus's
    /// controller configuration by, for that body and any body whose bus repeats it.
    pub embodiment: &'static str,
    /// The registry's rendered bindings (`bindings/<body>.json`).
    pub bindings: &'static str,
    /// Each motor answers on its send id plus this.
    pub master_id_offset: u16,
    /// Every driven joint's gains, in the bindings' native state order.
    pub joints: &'static [JointGains],
    /// How long each enabled motor may hear no command before it disables itself
    /// ([`DamiaoAxis::can_timeout`]); `None` where the body's sources state none.
    pub can_timeout: Option<Duration>,
    /// Whether the motors' feedback state field reports enable and disable
    /// ([`DamiaoAxis::reports_enable_state`]).
    pub reports_enable_state: bool,
}

/// Every single-bus Damiao body this crate opens.
pub const BODIES: [&DamiaoBody; 2] = [
    &crate::humanoid::HUMANOID_UPPER_BODY,
    &crate::static_upper_body::STATIC_UPPER_BODY,
];

impl DamiaoBody {
    /// The body among [`BODIES`] whose bindings carry `embodiment`'s content id.
    ///
    /// # Errors
    ///
    /// Returns why a body's bindings do not parse.
    pub fn of(embodiment: &str) -> Result<Option<&'static Self>, BodyError> {
        for body in BODIES {
            if BodyBindings::parse(body.bindings)?.is(embodiment) {
                return Ok(Some(body));
            }
        }
        Ok(None)
    }

    /// The rendered body's Damiao axes, calibrated by `drive`, refused unless they are on one
    /// bus and are the joints configured here. A body with other buses (the static upper
    /// body's Feetech neck) leaves those to the crates that drive them.
    ///
    /// # Errors
    ///
    /// Refuses other joints, Damiao axes on more than one port, and a drive of another body.
    pub fn body(&self, drive: &InstanceDrive) -> Result<BodyBindings, BodyError> {
        self.configured(BodyBindings::parse(self.bindings)?.calibrated(drive)?)
    }

    /// `body`'s Damiao axes, refused unless they are on one bus and are the joints configured
    /// here: this body's own, or a composed body's bus named as its twin here
    /// (`BodyBindings::as_twin`).
    fn configured(&self, body: BodyBindings) -> Result<BodyBindings, BodyError> {
        let body = body.on(DAMIAO_CAN);
        let joints: Vec<&str> = self.joints.iter().map(|gains| gains.joint).collect();
        body.require_joints(&joints)?;
        body.one_bus()?;
        Ok(body)
    }

    /// The body among [`BODIES`] the registry names `body`: the configuration its bus, or the
    /// bus of a body that repeats it, is opened with.
    #[must_use]
    pub fn named(body: &str) -> Option<&'static Self> {
        BODIES
            .into_iter()
            .find(|candidate| candidate.embodiment == body)
    }

    /// Every motor of `twin`, a composed body's bus named as this body
    /// (`BodyBindings::as_twin`), configured as this body's, in its order.
    ///
    /// # Errors
    ///
    /// Refuses joints that are not the ones configured here.
    pub fn axes_of(&self, twin: BodyBindings) -> Result<Vec<DamiaoAxis>, BodyError> {
        Ok(self
            .configured(twin)?
            .axes
            .into_iter()
            .zip(self.joints)
            .map(|(axis, gains)| self.axis(axis, gains))
            .collect())
    }

    /// Every motor, in native state order, as `drive`'s robot drives it.
    ///
    /// # Errors
    ///
    /// Refuses rendered bindings whose joints are not the ones configured here, and a drive of
    /// another body.
    pub fn axes(&self, drive: &InstanceDrive) -> Result<Vec<DamiaoAxis>, BodyError> {
        Ok(self
            .body(drive)?
            .axes
            .into_iter()
            .zip(self.joints)
            .map(|(axis, gains)| self.axis(axis, gains))
            .collect())
    }

    /// Open every motor on `port`, each brought up as the vendor does and left disabled.
    ///
    /// # Errors
    ///
    /// Returns why the chain did not open.
    pub fn open<P: CanPort>(
        &self,
        port: P,
        drive: &InstanceDrive,
    ) -> Result<DamiaoChain<P>, DamiaoChainError> {
        DamiaoChain::open(port, self.axes(drive)?)
    }

    fn axis(&self, axis: BoundAxis, gains: &JointGains) -> DamiaoAxis {
        // The motor's range is the joint's bounds through this unit's drive map, so a
        // calibrated zero or direction carries it with the joint.
        let ends = [
            axis.actuator_from_joint(axis.lower),
            axis.actuator_from_joint(axis.upper),
        ];
        DamiaoAxis {
            master_id: axis.binding.bus_id + self.master_id_offset,
            motor_lower: ends[0].min(ends[1]),
            motor_upper: ends[0].max(ends[1]),
            axis,
            kp: gains.kp,
            kd: gains.kd,
            can_timeout: self.can_timeout,
            reports_enable_state: self.reports_enable_state,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_bodys_gains_name_its_bindings_joints_in_order() {
        for body in BODIES {
            let bindings = BodyBindings::parse(body.bindings).expect("bindings");
            let joints: Vec<&str> = body.joints.iter().map(|gains| gains.joint).collect();
            let damiao = bindings.clone().on(DAMIAO_CAN);
            damiao.require_joints(&joints).expect("joints");
            assert_eq!(damiao.one_bus().expect("one bus").port, "can");
            assert_eq!(
                DamiaoBody::of(&bindings.id)
                    .expect("parses")
                    .map(|found| found.bindings),
                Some(body.bindings)
            );
        }
        assert!(DamiaoBody::of("another").expect("parses").is_none());
    }
}
