//! UVC controls the rig drives: manual exposure (camera terminal), gain (processing unit),
//! and the Sonix extension unit the flash UID is read through. Each request is a class
//! request to an entity: `wValue = selector << 8`, `wIndex = entity << 8 | interface`.
//!
//! V4L2's `auto_exposure = 1 (Manual Mode)` is UVC `CT_AE_MODE_CONTROL = 0x01`,
//! `exposure_time_absolute` is `CT_EXPOSURE_TIME_ABSOLUTE_CONTROL` (100 µs units), and
//! `gain` is `PU_GAIN_CONTROL` — so the laptop pod's settings and these are the same numbers.
//!
//! A colour preset (white balance, saturation, contrast, …: the processing unit's other
//! controls) is opt-in: written once when the camera opens, before streaming starts, and
//! never per frame. Without one the camera keeps whatever it holds, as before.

use std::io;

use serde_json::{Map, Value, json};
use sx_yubi::camera::{ExposureConfig, Setting};
use sx_yubi::identity::{CAMERA_XU_SELECTOR, Refused, Xu};

use crate::usb::{CLASS_INTERFACE_IN, CLASS_INTERFACE_OUT, Control};

pub const SET_CUR: u8 = 0x01;
pub const GET_CUR: u8 = 0x81;
pub const CT_AE_MODE_CONTROL: u8 = 0x02;
pub const CT_EXPOSURE_TIME_ABSOLUTE_CONTROL: u8 = 0x04;
pub const PU_GAIN_CONTROL: u8 = 0x04;
pub const AE_MODE_MANUAL: u8 = 0x01;
pub const PU_BACKLIGHT_COMPENSATION_CONTROL: u8 = 0x01;
pub const PU_BRIGHTNESS_CONTROL: u8 = 0x02;
pub const PU_CONTRAST_CONTROL: u8 = 0x03;
pub const PU_POWER_LINE_FREQUENCY_CONTROL: u8 = 0x05;
pub const PU_HUE_CONTROL: u8 = 0x06;
pub const PU_SATURATION_CONTROL: u8 = 0x07;
pub const PU_SHARPNESS_CONTROL: u8 = 0x08;
pub const PU_GAMMA_CONTROL: u8 = 0x09;
pub const PU_WHITE_BALANCE_TEMPERATURE_CONTROL: u8 = 0x0a;
pub const PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL: u8 = 0x0b;

/// One processing-unit control a colour preset sets: its V4L2-style name, UVC selector,
/// wire size (1 or 2 bytes, little-endian) and whether the camera reads it as signed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ColourControl {
    pub name: &'static str,
    pub selector: u8,
    pub size: u8,
    pub signed: bool,
    pub value: i16,
}

/// A named set of processing-unit controls, written in order (white balance auto off before
/// its temperature) once at camera open.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ColourPreset {
    pub name: &'static str,
    pub controls: &'static [ColourControl],
}

const fn pu(name: &'static str, selector: u8, size: u8, signed: bool, value: i16) -> ColourControl {
    ColourControl {
        name,
        selector,
        size,
        signed,
        value,
    }
}

/// The gripper colour calibration point: 3100 K fixed white balance, saturation 60,
/// 50 Hz anti-flicker, everything else at the camera's documented defaults.
pub const GRIPPER_3100K_V1: ColourPreset = ColourPreset {
    name: "gripper_3100k_v1",
    controls: &[
        pu(
            "white_balance_temperature_auto",
            PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL,
            1,
            false,
            0,
        ),
        pu(
            "white_balance_temperature",
            PU_WHITE_BALANCE_TEMPERATURE_CONTROL,
            2,
            false,
            3100,
        ),
        pu("saturation", PU_SATURATION_CONTROL, 2, false, 60),
        pu("contrast", PU_CONTRAST_CONTROL, 2, false, 32),
        pu("brightness", PU_BRIGHTNESS_CONTROL, 2, true, 0),
        pu("hue", PU_HUE_CONTROL, 2, true, 0),
        pu("gamma", PU_GAMMA_CONTROL, 2, false, 100),
        pu("sharpness", PU_SHARPNESS_CONTROL, 2, false, 2),
        pu(
            "power_line_frequency",
            PU_POWER_LINE_FREQUENCY_CONTROL,
            1,
            false,
            1,
        ),
        pu(
            "backlight_compensation",
            PU_BACKLIGHT_COMPENSATION_CONTROL,
            2,
            false,
            1,
        ),
    ],
};

/// Every preset a rig config may name.
pub const COLOUR_PRESETS: [&ColourPreset; 1] = [&GRIPPER_3100K_V1];

/// The preset a rig config names, or `None` for an unknown name (the caller refuses it).
pub fn colour_preset(name: &str) -> Option<&'static ColourPreset> {
    COLOUR_PRESETS
        .into_iter()
        .find(|preset| preset.name == name)
}

/// What a camera's receipt says about colour when no preset was asked for: nothing was
/// written, the camera holds whatever it held.
pub fn colour_untouched() -> Value {
    json!({"preset": Value::Null})
}

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

/// Write each of a preset's controls and read it back, once, at camera open. A control the
/// camera refuses (or answers short) is named in `refused` and the rest still go out, so the
/// stream opens and the receipt states exactly what the camera holds; `matched` is true only
/// when every control reads back as asked.
pub fn apply_colour_preset(
    control: &impl Control,
    entities: &Entities,
    preset: &ColourPreset,
) -> Value {
    let mut held = Map::new();
    let mut refused = Map::new();
    let mut requested = Map::new();
    for c in preset.controls {
        requested.insert(c.name.to_owned(), json!(c.value));
        let unit = entities.processing_unit;
        let answer = match c.size {
            1 => set(control, entities, unit, c.selector, &[c.value as u8])
                .and_then(|()| get::<1>(control, entities, unit, c.selector))
                .map(|[byte]| i32::from(byte)),
            2 => set(control, entities, unit, c.selector, &c.value.to_le_bytes())
                .and_then(|()| get::<2>(control, entities, unit, c.selector))
                .map(|bytes| {
                    if c.signed {
                        i32::from(i16::from_le_bytes(bytes))
                    } else {
                        i32::from(u16::from_le_bytes(bytes))
                    }
                }),
            size => Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("unsupported control size {size}"),
            )),
        };
        match answer {
            Ok(value) => {
                held.insert(c.name.to_owned(), json!(value));
            }
            Err(error) => {
                refused.insert(c.name.to_owned(), json!(error.to_string()));
            }
        }
    }
    let matched = refused.is_empty()
        && preset
            .controls
            .iter()
            .all(|c| held.get(c.name) == Some(&json!(c.value)));
    json!({
        "preset": preset.name,
        "requested": requested,
        "held": held,
        "refused": refused,
        "matched": matched,
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
    fn colour_preset_writes_each_control_once_to_the_processing_unit_and_reads_it_back() {
        let controls = Controls::default();
        let status = apply_colour_preset(&controls, &ENTITIES, &GRIPPER_3100K_V1);
        assert_eq!(status["preset"], "gripper_3100k_v1");
        assert_eq!(status["matched"], true, "{status}");
        assert_eq!(status["held"]["white_balance_temperature"], 3100);
        assert_eq!(status["held"]["saturation"], 60);
        assert_eq!(status["held"]["power_line_frequency"], 1);
        assert!(status["refused"].as_object().unwrap().is_empty());
        let log = controls.log.borrow();
        let writes: Vec<&Request> = log.iter().filter(|entry| entry.1 == SET_CUR).collect();
        assert_eq!(writes.len(), GRIPPER_3100K_V1.controls.len());
        // White balance auto goes off (1 byte) before the temperature (2 bytes LE) is set.
        assert_eq!(*writes[0], (0x21, 0x01, 0x0b00, 0x0200, vec![0]));
        assert_eq!(*writes[1], (0x21, 0x01, 0x0a00, 0x0200, vec![0x1c, 0x0c]));
        assert_eq!(*writes[8], (0x21, 0x01, 0x0500, 0x0200, vec![1]));
        assert!(
            log.iter().all(|entry| entry.3 == 0x0200),
            "only the processing unit is addressed"
        );
        // Exposure and gain are not the preset's to touch.
        assert!(
            log.iter()
                .all(|entry| entry.2 != u16::from(PU_GAIN_CONTROL) << 8)
        );
    }

    #[test]
    fn signed_controls_round_trip_negative_values() {
        static NEGATIVE: ColourPreset = ColourPreset {
            name: "test_negative",
            controls: &[pu("brightness", PU_BRIGHTNESS_CONTROL, 2, true, -5)],
        };
        let controls = Controls::default();
        let status = apply_colour_preset(&controls, &ENTITIES, &NEGATIVE);
        assert_eq!(status["held"]["brightness"], -5);
        assert_eq!(status["matched"], true);
        assert_eq!(controls.log.borrow()[0].4, vec![0xfb, 0xff]);
    }

    /// A camera that stalls one selector: the rest still go out and the receipt names it.
    struct Refusing {
        inner: Controls,
        selector: u8,
    }

    impl Control for Refusing {
        fn control_in(
            &self,
            request_type: u8,
            request: u8,
            value: u16,
            index: u16,
            data: &mut [u8],
        ) -> io::Result<usize> {
            self.inner
                .control_in(request_type, request, value, index, data)
        }

        fn control_out(
            &self,
            request_type: u8,
            request: u8,
            value: u16,
            index: u16,
            data: &[u8],
        ) -> io::Result<()> {
            if value == u16::from(self.selector) << 8 {
                return Err(io::Error::from(io::ErrorKind::BrokenPipe));
            }
            self.inner
                .control_out(request_type, request, value, index, data)
        }
    }

    #[test]
    fn a_refused_colour_control_is_reported_not_fatal() {
        let camera = Refusing {
            inner: Controls::default(),
            selector: PU_HUE_CONTROL,
        };
        let status = apply_colour_preset(&camera, &ENTITIES, &GRIPPER_3100K_V1);
        assert_eq!(status["matched"], false);
        assert!(status["refused"]["hue"].is_string(), "{status}");
        assert_eq!(status["held"]["saturation"], 60);
        assert!(status["held"].get("hue").is_none());
    }

    #[test]
    fn presets_are_found_by_name_only() {
        assert_eq!(colour_preset("gripper_3100k_v1"), Some(&GRIPPER_3100K_V1));
        assert_eq!(colour_preset("GRIPPER_3100K_V1"), None);
        assert_eq!(colour_preset(""), None);
        assert_eq!(colour_untouched(), json!({"preset": null}));
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
