//! UVC controls the rig drives: manual exposure (camera terminal), gain (processing unit),
//! and the Sonix extension unit the flash UID is read through. Each request is a class
//! request to an entity: `wValue = selector << 8`, `wIndex = entity << 8 | interface`.
//!
//! V4L2's `auto_exposure = 1 (Manual Mode)` is UVC `CT_AE_MODE_CONTROL = 0x01`,
//! `exposure_time_absolute` is `CT_EXPOSURE_TIME_ABSOLUTE_CONTROL` (100 µs units), and
//! `gain` is `PU_GAIN_CONTROL` — so the laptop pod's settings and these are the same numbers.

use std::io;

use sx_yubi::camera::{ExposureConfig, Setting};
use sx_yubi::identity::{CAMERA_XU_SELECTOR, Refused, Xu};

use crate::usb::{CLASS_INTERFACE_IN, CLASS_INTERFACE_OUT, Control};

pub const SET_CUR: u8 = 0x01;
pub const GET_CUR: u8 = 0x81;
pub const CT_AE_MODE_CONTROL: u8 = 0x02;
pub const CT_EXPOSURE_TIME_ABSOLUTE_CONTROL: u8 = 0x04;
pub const PU_GAIN_CONTROL: u8 = 0x04;
pub const AE_MODE_MANUAL: u8 = 0x01;

/// Where the rig's controls live on one camera.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Entities {
    pub interface: u8,
    pub camera_terminal: u8,
    pub processing_unit: u8,
    pub extension_unit: u8,
}

fn index(entity: u8, interface: u8) -> u16 {
    (u16::from(entity) << 8) | u16::from(interface)
}

fn set(
    control: &impl Control,
    entities: &Entities,
    entity: u8,
    selector: u8,
    bytes: &[u8],
) -> io::Result<()> {
    control.control_out(
        CLASS_INTERFACE_OUT,
        SET_CUR,
        u16::from(selector) << 8,
        index(entity, entities.interface),
        bytes,
    )
}

fn get<const N: usize>(
    control: &impl Control,
    entities: &Entities,
    entity: u8,
    selector: u8,
) -> io::Result<[u8; N]> {
    let mut bytes = [0; N];
    let read = control.control_in(
        CLASS_INTERFACE_IN,
        GET_CUR,
        u16::from(selector) << 8,
        index(entity, entities.interface),
        &mut bytes,
    )?;
    if read != N {
        return Err(io::Error::new(
            io::ErrorKind::UnexpectedEof,
            format!("control answered {read} of {N} bytes"),
        ));
    }
    Ok(bytes)
}

/// Write exposure and gain, entering manual exposure first when asked, and prove the camera
/// holds what was asked: the laptop pod's `apply_exposure`, over usbfs.
pub fn apply_exposure(
    control: &impl Control,
    entities: &Entities,
    config: ExposureConfig,
    setting: Setting,
    enter_manual: bool,
) -> Result<Setting, String> {
    if !(config.min.exposure_100us..=config.max.exposure_100us).contains(&setting.exposure_100us)
        || !(config.min.gain..=config.max.gain).contains(&setting.gain)
    {
        return Err("camera exposure request is outside the accepted range".to_owned());
    }
    let failed = |error: io::Error| format!("camera exposure control: {error}");
    if enter_manual {
        set(
            control,
            entities,
            entities.camera_terminal,
            CT_AE_MODE_CONTROL,
            &[AE_MODE_MANUAL],
        )
        .map_err(failed)?;
    }
    set(
        control,
        entities,
        entities.camera_terminal,
        CT_EXPOSURE_TIME_ABSOLUTE_CONTROL,
        &(setting.exposure_100us as u32).to_le_bytes(),
    )
    .map_err(failed)?;
    set(
        control,
        entities,
        entities.processing_unit,
        PU_GAIN_CONTROL,
        &(setting.gain as u16).to_le_bytes(),
    )
    .map_err(failed)?;
    let [mode] = get::<1>(
        control,
        entities,
        entities.camera_terminal,
        CT_AE_MODE_CONTROL,
    )
    .map_err(failed)?;
    if mode != AE_MODE_MANUAL {
        return Err("camera did not hold manual exposure mode".to_owned());
    }
    let exposure = get::<4>(
        control,
        entities,
        entities.camera_terminal,
        CT_EXPOSURE_TIME_ABSOLUTE_CONTROL,
    )
    .map_err(failed)?;
    let gain =
        get::<2>(control, entities, entities.processing_unit, PU_GAIN_CONTROL).map_err(failed)?;
    Ok(Setting {
        exposure_100us: u32::from_le_bytes(exposure) as i32,
        gain: i32::from(u16::from_le_bytes(gain)),
    })
}

/// The Sonix register-access extension control over a usbfs control pipe, so the pod's one
/// flash-UID procedure (`sx_yubi::identity::flash_uid`) runs unchanged.
pub struct ExtensionUnit<'a, C: Control> {
    pub control: &'a C,
    pub entities: Entities,
}

#[derive(Debug)]
pub struct XuError(pub String);

impl From<Refused> for XuError {
    fn from(refused: Refused) -> Self {
        Self(refused.0)
    }
}

impl std::fmt::Display for XuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl<C: Control> Xu for ExtensionUnit<'_, C> {
    type Error = XuError;

    fn query(&self, query: u8, bytes: &mut [u8; 4]) -> Result<(), XuError> {
        let failed = |error: io::Error| XuError(format!("extension unit: {error}"));
        if query == SET_CUR {
            set(
                self.control,
                &self.entities,
                self.entities.extension_unit,
                CAMERA_XU_SELECTOR,
                bytes,
            )
            .map_err(failed)
        } else {
            *bytes = get::<4>(
                self.control,
                &self.entities,
                self.entities.extension_unit,
                CAMERA_XU_SELECTOR,
            )
            .map_err(failed)?;
            Ok(())
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::collections::BTreeMap;

    /// (bmRequestType, bRequest, wValue, wIndex, data out).
    pub(crate) type Request = (u8, u8, u16, u16, Vec<u8>);

    /// A camera's control endpoint: SET_CUR stores, GET_CUR answers what was stored.
    #[derive(Default)]
    pub(crate) struct Controls {
        pub(crate) state: RefCell<BTreeMap<(u16, u16), Vec<u8>>>,
        pub(crate) log: RefCell<Vec<Request>>,
        pub(crate) hold_auto: bool,
    }

    impl Control for Controls {
        fn control_in(
            &self,
            request_type: u8,
            request: u8,
            value: u16,
            index: u16,
            data: &mut [u8],
        ) -> io::Result<usize> {
            self.log
                .borrow_mut()
                .push((request_type, request, value, index, Vec::new()));
            let state = self.state.borrow();
            let stored = state
                .get(&(value, index))
                .ok_or_else(|| io::Error::from(io::ErrorKind::BrokenPipe))?;
            let mut stored = stored.clone();
            if self.hold_auto && value == u16::from(CT_AE_MODE_CONTROL) << 8 {
                stored = vec![0x08];
            }
            data[..stored.len()].copy_from_slice(&stored);
            Ok(stored.len())
        }

        fn control_out(
            &self,
            request_type: u8,
            request: u8,
            value: u16,
            index: u16,
            data: &[u8],
        ) -> io::Result<()> {
            self.log
                .borrow_mut()
                .push((request_type, request, value, index, data.to_vec()));
            self.state
                .borrow_mut()
                .insert((value, index), data.to_vec());
            Ok(())
        }
    }

    const CONTROLS: ExposureConfig = ExposureConfig {
        start: Setting {
            exposure_100us: 70,
            gain: 0,
        },
        min: Setting {
            exposure_100us: 10,
            gain: 0,
        },
        max: Setting {
            exposure_100us: 70,
            gain: 100,
        },
    };

    const ENTITIES: Entities = Entities {
        interface: 0,
        camera_terminal: 1,
        processing_unit: 2,
        extension_unit: 3,
    };

    #[test]
    fn exposure_requests_address_the_spec_entities_and_are_read_back() {
        let controls = Controls::default();
        let held = apply_exposure(&controls, &ENTITIES, CONTROLS, CONTROLS.start, true).unwrap();
        assert_eq!(held, CONTROLS.start);
        let log = controls.log.borrow();
        assert_eq!(log[0], (0x21, 0x01, 0x0200, 0x0100, vec![0x01]));
        assert_eq!(log[1], (0x21, 0x01, 0x0400, 0x0100, vec![70, 0, 0, 0]));
        assert_eq!(log[2], (0x21, 0x01, 0x0400, 0x0200, vec![0, 0]));
        assert!(
            log[3..]
                .iter()
                .all(|entry| entry.0 == 0xa1 && entry.1 == 0x81)
        );
    }

    #[test]
    fn out_of_contract_settings_and_a_camera_left_in_auto_are_refused() {
        let controls = Controls::default();
        for setting in [
            Setting {
                exposure_100us: 71,
                gain: 0,
            },
            Setting {
                exposure_100us: 70,
                gain: 101,
            },
        ] {
            assert!(apply_exposure(&controls, &ENTITIES, CONTROLS, setting, true).is_err());
        }
        assert!(
            controls.log.borrow().is_empty(),
            "nothing reaches the camera"
        );
        let stubborn = Controls {
            hold_auto: true,
            ..Controls::default()
        };
        let error =
            apply_exposure(&stubborn, &ENTITIES, CONTROLS, CONTROLS.start, true).unwrap_err();
        assert!(error.contains("manual"), "{error}");
    }

    #[test]
    fn caller_supplied_limits_are_checked_before_device_io() {
        let controls = Controls::default();
        let bounded = ExposureConfig {
            min: Setting {
                exposure_100us: 20,
                gain: 10,
            },
            max: Setting {
                exposure_100us: 30,
                gain: 20,
            },
            start: Setting {
                exposure_100us: 25,
                gain: 15,
            },
        };
        for setting in [
            Setting {
                exposure_100us: 19,
                gain: 15,
            },
            Setting {
                exposure_100us: 31,
                gain: 15,
            },
            Setting {
                exposure_100us: 25,
                gain: 9,
            },
            Setting {
                exposure_100us: 25,
                gain: 21,
            },
        ] {
            assert!(apply_exposure(&controls, &ENTITIES, bounded, setting, true).is_err());
        }
        assert!(controls.log.borrow().is_empty());
        assert_eq!(
            apply_exposure(&controls, &ENTITIES, bounded, bounded.start, true).unwrap(),
            bounded.start
        );
    }

    #[test]
    fn extension_unit_queries_reach_unit_3_selector_1() {
        let controls = Controls::default();
        let xu = ExtensionUnit {
            control: &controls,
            entities: ENTITIES,
        };
        let mut bytes = [0x1f, 0x10, 0, 0xff];
        xu.query(SET_CUR, &mut bytes).unwrap();
        let mut read = [0; 4];
        xu.query(GET_CUR, &mut read).unwrap();
        assert_eq!(read, [0x1f, 0x10, 0, 0xff]);
        assert_eq!(
            controls.log.borrow()[0],
            (0x21, 0x01, 0x0100, 0x0300, bytes.to_vec())
        );
    }
}
