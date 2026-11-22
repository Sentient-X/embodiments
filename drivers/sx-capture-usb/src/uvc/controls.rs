//! UVC controls the rig drives: manual exposure (camera terminal), gain (processing unit),
//! and the Sonix extension unit the flash UID is read through. Each request is a class
//! request to an entity: `wValue = selector << 8`, `wIndex = entity << 8 | interface`.
//!
//! V4L2's `auto_exposure = 1 (Manual Mode)` is UVC `CT_AE_MODE_CONTROL = 0x01`,
//! `exposure_time_absolute` is `CT_EXPOSURE_TIME_ABSOLUTE_CONTROL` (100 µs units), and
//! `gain` is `PU_GAIN_CONTROL` — so the laptop pod's settings and these are the same numbers.
//!
//! A colour preset ([`ColourPreset`]: manual exposure, exposure time and gain, white balance,
//! saturation, contrast, …) is opt-in: written once when the camera opens, before streaming
//! starts, read back, and never touched per frame. On a normal open the driver enables
//! firmware auto white balance once. Exposure and gain still use the caller's bounds.

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

pub const CT_AE_PRIORITY_CONTROL: u8 = 0x03;

/// The UVC entity a control is addressed to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Unit {
    CameraTerminal,
    ProcessingUnit,
}

/// One control a colour preset sets: its name as the colour bundle (and V4L2) names it, the
/// UVC entity and selector, its wire size (1, 2 or 4 bytes, little-endian), whether the
/// camera reads it as signed, and the value in the bundle's terms.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ColourControl {
    pub name: &'static str,
    pub unit: Unit,
    pub selector: u8,
    pub size: u8,
    pub signed: bool,
    pub value: i32,
}

/// A per-camera locked colour preset (`sx.gripper-colour-profile/v1`'s `preset` and
/// `preset_conditions`): every value is written once when the camera opens, before streaming,
/// and read back. Exposure is manual and held at `exposure_100us`/`gain`: a camera opened with
/// a preset is never steered by an exposure controller.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ColourPreset {
    /// The bundle's `profile_name`, named in every receipt.
    pub name: String,
    /// V4L2 `auto_exposure`: only 1 (manual) is accepted.
    pub auto_exposure: i32,
    /// `exposure_time_absolute`, 100 µs units.
    pub exposure_100us: i32,
    pub gain: i32,
    /// V4L2 `white_balance_temperature_auto`: 0 fixes `wb_temperature`, 1 leaves it unwritten.
    pub auto_wb: i32,
    pub wb_temperature: i32,
    pub brightness: i32,
    pub contrast: i32,
    pub saturation: i32,
    pub hue: i32,
    pub gamma: i32,
    pub sharpness: i32,
    pub power_line_frequency: Option<i32>,
    pub backlight_compensation: Option<i32>,
    /// V4L2 `exposure_dynamic_framerate`, UVC `CT_AE_PRIORITY_CONTROL`.
    pub exposure_dynamic_framerate: Option<i32>,
    /// A control the camera refuses or reads back otherwise fails the open (true) or is only
    /// recorded (false).
    pub refuse_on_mismatch: bool,
}

/// The accepted range of each preset value on the YUBI wrist camera (32e4:9230).
pub const PRESET_RANGES: [(&str, i32, i32); 14] = [
    ("auto_exposure", 1, 1),
    ("exposure", 1, 5000),
    ("gain", 0, 100),
    ("auto_wb", 0, 1),
    ("wb_temperature", 2800, 6500),
    ("brightness", -64, 64),
    ("contrast", 0, 64),
    ("saturation", 0, 128),
    ("hue", -40, 40),
    ("gamma", 72, 500),
    ("sharpness", 0, 6),
    ("power_line_frequency", 0, 2),
    ("backlight_compensation", 0, 2),
    ("exposure_dynamic_framerate", 0, 1),
];

const fn control(
    name: &'static str,
    unit: Unit,
    selector: u8,
    size: u8,
    signed: bool,
    value: i32,
) -> ColourControl {
    ColourControl {
        name,
        unit,
        selector,
        size,
        signed,
        value,
    }
}

impl ColourPreset {
    /// Every value by its bundle name, `None` for a condition the preset leaves alone.
    fn values(&self) -> [(&'static str, Option<i32>); 14] {
        [
            ("auto_exposure", Some(self.auto_exposure)),
            ("exposure", Some(self.exposure_100us)),
            ("gain", Some(self.gain)),
            ("auto_wb", Some(self.auto_wb)),
            ("wb_temperature", Some(self.wb_temperature)),
            ("brightness", Some(self.brightness)),
            ("contrast", Some(self.contrast)),
            ("saturation", Some(self.saturation)),
            ("hue", Some(self.hue)),
            ("gamma", Some(self.gamma)),
            ("sharpness", Some(self.sharpness)),
            ("power_line_frequency", self.power_line_frequency),
            ("backlight_compensation", self.backlight_compensation),
            (
                "exposure_dynamic_framerate",
                self.exposure_dynamic_framerate,
            ),
        ]
    }

    /// Refuse a preset the camera cannot hold: no name, auto exposure, or any value outside
    /// [`PRESET_RANGES`] (every out-of-range value is named).
    pub fn validate(&self) -> Result<(), String> {
        if self.name.trim().is_empty() {
            return Err("colour preset has no name".to_owned());
        }
        let outside: Vec<String> = self
            .values()
            .into_iter()
            .zip(PRESET_RANGES)
            .filter_map(|((name, value), (_, min, max))| {
                let value = value?;
                (!(min..=max).contains(&value))
                    .then(|| format!("{name} {value} not in {min}..={max}"))
            })
            .collect();
        if outside.is_empty() {
            Ok(())
        } else {
            Err(format!(
                "colour preset {}: {}",
                self.name,
                outside.join(", ")
            ))
        }
    }

    /// The exposure the camera is held at: start = min = max, so nothing may move it.
    pub fn exposure_config(&self) -> ExposureConfig {
        let held = Setting {
            exposure_100us: self.exposure_100us,
            gain: self.gain,
        };
        ExposureConfig {
            start: held,
            min: held,
            max: held,
        }
    }

    /// The controls in the order they are written: manual exposure before its time and gain,
    /// white balance auto off before its temperature (which is not written under auto).
    pub fn controls(&self) -> Vec<ColourControl> {
        use Unit::{CameraTerminal as Ct, ProcessingUnit as Pu};
        let mut out = vec![control(
            "auto_exposure",
            Ct,
            CT_AE_MODE_CONTROL,
            1,
            false,
            self.auto_exposure,
        )];
        if let Some(value) = self.exposure_dynamic_framerate {
            out.push(control(
                "exposure_dynamic_framerate",
                Ct,
                CT_AE_PRIORITY_CONTROL,
                1,
                false,
                value,
            ));
        }
        out.extend([
            control(
                "exposure",
                Ct,
                CT_EXPOSURE_TIME_ABSOLUTE_CONTROL,
                4,
                false,
                self.exposure_100us,
            ),
            control("gain", Pu, PU_GAIN_CONTROL, 2, false, self.gain),
            control(
                "auto_wb",
                Pu,
                PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL,
                1,
                false,
                self.auto_wb,
            ),
        ]);
        if self.auto_wb == 0 {
            out.push(control(
                "wb_temperature",
                Pu,
                PU_WHITE_BALANCE_TEMPERATURE_CONTROL,
                2,
                false,
                self.wb_temperature,
            ));
        }
        out.extend([
            control(
                "brightness",
                Pu,
                PU_BRIGHTNESS_CONTROL,
                2,
                true,
                self.brightness,
            ),
            control("contrast", Pu, PU_CONTRAST_CONTROL, 2, false, self.contrast),
            control(
                "saturation",
                Pu,
                PU_SATURATION_CONTROL,
                2,
                false,
                self.saturation,
            ),
            control("hue", Pu, PU_HUE_CONTROL, 2, true, self.hue),
            control("gamma", Pu, PU_GAMMA_CONTROL, 2, false, self.gamma),
            control(
                "sharpness",
                Pu,
                PU_SHARPNESS_CONTROL,
                2,
                false,
                self.sharpness,
            ),
        ]);
        if let Some(value) = self.power_line_frequency {
            out.push(control(
                "power_line_frequency",
                Pu,
                PU_POWER_LINE_FREQUENCY_CONTROL,
                1,
                false,
                value,
            ));
        }
        if let Some(value) = self.backlight_compensation {
            out.push(control(
                "backlight_compensation",
                Pu,
                PU_BACKLIGHT_COMPENSATION_CONTROL,
                2,
                false,
                value,
            ));
        }
        out
    }

    /// The receipt of one application, with separate write and read results for every control.
    fn report(
        &self,
        results: impl IntoIterator<Item = (ColourControl, io::Result<()>, io::Result<i32>)>,
    ) -> Value {
        let mut requested = Map::new();
        let mut read_back = Map::new();
        let mut refused = Map::new();
        let mut mismatched = Vec::new();
        for (c, write, read) in results {
            requested.insert(c.name.to_owned(), json!(c.value));
            let mut errors = Vec::new();
            if let Err(error) = write {
                errors.push(format!("SET_CUR: {error}"));
            }
            match read {
                Ok(value) => {
                    if value != c.value {
                        mismatched.push(json!(c.name));
                    }
                    read_back.insert(c.name.to_owned(), json!(value));
                }
                Err(error) => errors.push(format!("GET_CUR: {error}")),
            }
            if !errors.is_empty() {
                refused.insert(c.name.to_owned(), json!(errors.join("; ")));
            }
        }
        let matched = refused.is_empty() && mismatched.is_empty();
        json!({
            "preset": self.name,
            "requested": requested,
            "read_back": read_back,
            "refused": refused,
            "mismatched": mismatched,
            "matched": matched,
            "refuse_on_mismatch": self.refuse_on_mismatch,
        })
    }

    /// Why the camera may not stream under this preset: some control did not hold and the
    /// preset refuses a mismatch. `None` when every control held, or mismatches are only
    /// recorded.
    pub fn refusal(&self, report: &Value) -> Option<String> {
        if !self.refuse_on_mismatch || report["matched"] == json!(true) {
            return None;
        }
        Some(format!(
            "camera did not hold colour preset {}: mismatched {}, refused {}, read back {}",
            self.name, report["mismatched"], report["refused"], report["read_back"]
        ))
    }
}

/// V4L2's `auto_exposure` for a UVC `CT_AE_MODE_CONTROL` bitmap (manual 1, auto 0, shutter
/// priority 2, aperture priority 3); an unknown mode reads back as its negated bitmap.
fn ae_mode_v4l2(mode: u8) -> i32 {
    match mode {
        0x01 => 1,
        0x02 => 0,
        0x04 => 2,
        0x08 => 3,
        other => -i32::from(other),
    }
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
    check_exposure(config, setting)?;
    write_exposure(control, entities, setting, enter_manual)
}

fn check_exposure(config: ExposureConfig, setting: Setting) -> Result<(), String> {
    if !(config.min.exposure_100us..=config.max.exposure_100us).contains(&setting.exposure_100us)
        || !(config.min.gain..=config.max.gain).contains(&setting.gain)
    {
        return Err("camera exposure request is outside the accepted range".to_owned());
    }
    Ok(())
}

fn write_exposure(
    control: &impl Control,
    entities: &Entities,
    setting: Setting,
    enter_manual: bool,
) -> Result<Setting, String> {
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

/// Enable firmware white balance, establish bounded manual exposure, then verify white
/// balance after the last write. A white balance refusal is reported without stopping capture.
pub fn apply_capture_controls(
    control: &impl Control,
    entities: &Entities,
    config: ExposureConfig,
) -> Result<(Setting, Result<(), String>), String> {
    check_exposure(config, config.start)?;
    let unit = entities.processing_unit;
    let selector = PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL;
    let requested = set(control, entities, unit, selector, &[1]);
    let exposure = write_exposure(control, entities, config.start, true)?;
    let auto_white_balance = requested
        .map_err(|error| format!("SET_CUR: {error}"))
        .and_then(|()| {
            let [enabled] = get::<1>(control, entities, unit, selector)
                .map_err(|error| format!("GET_CUR: {error}"))?;
            if enabled == 1 {
                Ok(())
            } else {
                Err(format!(
                    "expected auto white balance 1, read back {enabled}"
                ))
            }
        });
    Ok((exposure, auto_white_balance))
}

/// Write a preset once at camera open, then read every control back before starting the stream.
/// A later write may affect an earlier control, so the receipt is based on the complete set
/// of writes. A failed write still gets a readback and is named in the report's `refused`.
pub fn apply_colour_preset(
    control: &impl Control,
    entities: &Entities,
    preset: &ColourPreset,
) -> Value {
    let controls = preset.controls();
    let mut writes = Vec::with_capacity(controls.len());
    for c in controls {
        let unit = match c.unit {
            Unit::CameraTerminal => entities.camera_terminal,
            Unit::ProcessingUnit => entities.processing_unit,
        };
        let manual_exposure = c.unit == Unit::CameraTerminal && c.selector == CT_AE_MODE_CONTROL;
        let value = if manual_exposure {
            i32::from(AE_MODE_MANUAL)
        } else {
            c.value
        };
        let bytes = value.to_le_bytes();
        let write = set(
            control,
            entities,
            unit,
            c.selector,
            &bytes[..usize::from(c.size)],
        );
        writes.push((c, unit, manual_exposure, write));
    }

    preset.report(writes.into_iter().map(|(c, unit, manual_exposure, write)| {
        let read = match c.size {
            1 if manual_exposure => {
                get::<1>(control, entities, unit, c.selector).map(|[mode]| ae_mode_v4l2(mode))
            }
            1 => get::<1>(control, entities, unit, c.selector).map(|[byte]| i32::from(byte)),
            2 if c.signed => get::<2>(control, entities, unit, c.selector)
                .map(|bytes| i32::from(i16::from_le_bytes(bytes))),
            2 => get::<2>(control, entities, unit, c.selector)
                .map(|bytes| i32::from(u16::from_le_bytes(bytes))),
            4 => get::<4>(control, entities, unit, c.selector)
                .map(|bytes| u32::from_le_bytes(bytes) as i32),
            size => Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("unsupported control size {size}"),
            )),
        };
        (c, write, read)
    }))
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
        hold_white_balance_off: bool,
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
            if self.hold_white_balance_off
                && value == u16::from(PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL) << 8
            {
                stored = vec![0];
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
    fn firmware_white_balance_readback_follows_the_manual_exposure_setup() {
        let controls = Controls::default();
        let (exposure, white_balance) =
            apply_capture_controls(&controls, &ENTITIES, CONTROLS).unwrap();
        assert_eq!(exposure, CONTROLS.start);
        assert_eq!(white_balance, Ok(()));
        let log = controls.log.borrow();
        assert_eq!(log.len(), 8);
        assert_eq!(log[0], (0x21, SET_CUR, 0x0b00, 0x0200, vec![1]));
        assert_eq!(log[1], (0x21, SET_CUR, 0x0200, 0x0100, vec![1]));
        assert_eq!(log[7], (0xa1, GET_CUR, 0x0b00, 0x0200, vec![]));
    }

    #[test]
    fn firmware_white_balance_requires_enabled_readback() {
        let controls = Controls {
            hold_white_balance_off: true,
            ..Controls::default()
        };
        let (exposure, white_balance) =
            apply_capture_controls(&controls, &ENTITIES, CONTROLS).unwrap();
        assert_eq!(exposure, CONTROLS.start);
        assert_eq!(
            white_balance,
            Err("expected auto white balance 1, read back 0".to_owned())
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
            let invalid_start = ExposureConfig {
                start: setting,
                ..CONTROLS
            };
            assert!(apply_capture_controls(&controls, &ENTITIES, invalid_start).is_err());
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

    /// The fleet bundle's preset (`fleet_lightC_2p3ms_wb3100_v1`).
    pub(crate) fn fleet_preset() -> ColourPreset {
        ColourPreset {
            name: "fleet_lightC_2p3ms_wb3100_v1".to_owned(),
            auto_exposure: 1,
            exposure_100us: 23,
            gain: 0,
            auto_wb: 0,
            wb_temperature: 3100,
            brightness: 0,
            contrast: 32,
            saturation: 60,
            hue: 0,
            gamma: 100,
            sharpness: 2,
            power_line_frequency: Some(1),
            backlight_compensation: Some(1),
            exposure_dynamic_framerate: Some(1),
            refuse_on_mismatch: true,
        }
    }

    #[test]
    fn a_preset_writes_each_control_once_to_its_unit_and_reads_it_back() {
        let controls = Controls::default();
        let preset = fleet_preset();
        let status = apply_colour_preset(&controls, &ENTITIES, &preset);
        assert_eq!(status["preset"], "fleet_lightC_2p3ms_wb3100_v1");
        assert_eq!(status["matched"], true, "{status}");
        assert_eq!(preset.refusal(&status), None);
        assert_eq!(status["read_back"]["auto_exposure"], 1);
        assert_eq!(status["read_back"]["exposure"], 23);
        assert_eq!(status["read_back"]["gain"], 0);
        assert_eq!(status["read_back"]["wb_temperature"], 3100);
        assert_eq!(status["read_back"]["saturation"], 60);
        assert_eq!(status["read_back"]["exposure_dynamic_framerate"], 1);
        assert_eq!(status["requested"], status["read_back"]);
        assert!(status["refused"].as_object().unwrap().is_empty());
        assert_eq!(status["mismatched"], json!([]));
        let log = controls.log.borrow();
        let writes: Vec<&Request> = log.iter().filter(|entry| entry.1 == SET_CUR).collect();
        assert_eq!(writes.len(), preset.controls().len());
        assert_eq!(writes.len(), 14);
        assert_eq!(
            log.iter().filter(|entry| entry.1 == GET_CUR).count(),
            writes.len(),
            "each control is read back once"
        );
        // Manual exposure, its priority, time (4 bytes) on the camera terminal; gain and the
        // colour controls on the processing unit; white balance auto off before its temperature.
        assert_eq!(*writes[0], (0x21, 0x01, 0x0200, 0x0100, vec![0x01]));
        assert_eq!(*writes[1], (0x21, 0x01, 0x0300, 0x0100, vec![0x01]));
        assert_eq!(*writes[2], (0x21, 0x01, 0x0400, 0x0100, vec![23, 0, 0, 0]));
        assert_eq!(*writes[3], (0x21, 0x01, 0x0400, 0x0200, vec![0, 0]));
        assert_eq!(*writes[4], (0x21, 0x01, 0x0b00, 0x0200, vec![0]));
        assert_eq!(*writes[5], (0x21, 0x01, 0x0a00, 0x0200, vec![0x1c, 0x0c]));
        assert_eq!(*writes[12], (0x21, 0x01, 0x0500, 0x0200, vec![1]));
        assert_eq!(*writes[13], (0x21, 0x01, 0x0100, 0x0200, vec![1, 0]));
        assert!(
            log.iter()
                .all(|entry| entry.3 == 0x0100 || entry.3 == 0x0200),
            "never the extension unit"
        );
    }

    #[test]
    fn auto_white_balance_leaves_the_temperature_unwritten_and_conditions_are_optional() {
        let controls = Controls::default();
        let preset = ColourPreset {
            auto_wb: 1,
            power_line_frequency: None,
            backlight_compensation: None,
            exposure_dynamic_framerate: None,
            ..fleet_preset()
        };
        let status = apply_colour_preset(&controls, &ENTITIES, &preset);
        assert_eq!(status["matched"], true, "{status}");
        assert!(status["requested"].get("wb_temperature").is_none());
        assert!(status["requested"].get("power_line_frequency").is_none());
        assert_eq!(preset.controls().len(), 10);
        assert!(
            controls
                .log
                .borrow()
                .iter()
                .all(|entry| entry.2 != u16::from(PU_WHITE_BALANCE_TEMPERATURE_CONTROL) << 8)
        );
    }

    #[test]
    fn signed_controls_round_trip_negative_values() {
        let controls = Controls::default();
        let preset = ColourPreset {
            brightness: -5,
            hue: -40,
            ..fleet_preset()
        };
        let status = apply_colour_preset(&controls, &ENTITIES, &preset);
        assert_eq!(status["read_back"]["brightness"], -5);
        assert_eq!(status["read_back"]["hue"], -40);
        assert_eq!(status["matched"], true);
        assert!(
            controls
                .log
                .borrow()
                .iter()
                .any(|entry| entry.2 == u16::from(PU_BRIGHTNESS_CONTROL) << 8
                    && entry.4 == vec![0xfb, 0xff])
        );
    }

    /// A camera that stalls one selector's write, or answers another with its own value.
    struct Quirky {
        inner: Controls,
        refuses: Option<u8>,
        /// (selector, the value GET_CUR answers).
        clamps: Option<(u8, u16)>,
        changes_gain_on_backlight: Option<u16>,
    }

    impl Control for Quirky {
        fn control_in(
            &self,
            request_type: u8,
            request: u8,
            value: u16,
            index: u16,
            data: &mut [u8],
        ) -> io::Result<usize> {
            let read = self
                .inner
                .control_in(request_type, request, value, index, data)?;
            if let Some((selector, answer)) = self.clamps
                && value == u16::from(selector) << 8
                && index == 0x0200
            {
                data[..2].copy_from_slice(&answer.to_le_bytes());
            }
            Ok(read)
        }

        fn control_out(
            &self,
            request_type: u8,
            request: u8,
            value: u16,
            index: u16,
            data: &[u8],
        ) -> io::Result<()> {
            if self
                .refuses
                .is_some_and(|s| value == u16::from(s) << 8 && index == 0x0200)
            {
                return Err(io::Error::from(io::ErrorKind::BrokenPipe));
            }
            self.inner
                .control_out(request_type, request, value, index, data)?;
            if let Some(gain) = self.changes_gain_on_backlight
                && value == u16::from(PU_BACKLIGHT_COMPENSATION_CONTROL) << 8
                && index == 0x0200
            {
                self.inner.state.borrow_mut().insert(
                    (u16::from(PU_GAIN_CONTROL) << 8, index),
                    gain.to_le_bytes().to_vec(),
                );
            }
            Ok(())
        }
    }

    #[test]
    fn a_later_control_write_is_reflected_in_the_final_readback() {
        let camera = Quirky {
            inner: Controls::default(),
            refuses: None,
            clamps: None,
            changes_gain_on_backlight: Some(40),
        };
        let preset = fleet_preset();
        let status = apply_colour_preset(&camera, &ENTITIES, &preset);
        assert_eq!(status["read_back"]["gain"], 40);
        assert_eq!(status["mismatched"], json!(["gain"]));
        assert!(preset.refusal(&status).is_some());
    }

    #[test]
    fn a_refused_or_mismatched_control_is_recorded_and_refuses_only_when_asked() {
        let camera = Quirky {
            inner: Controls::default(),
            refuses: Some(PU_HUE_CONTROL),
            clamps: Some((PU_SATURATION_CONTROL, 64)),
            changes_gain_on_backlight: None,
        };
        camera.inner.state.borrow_mut().insert(
            (u16::from(PU_HUE_CONTROL) << 8, 0x0200),
            (-12_i16).to_le_bytes().to_vec(),
        );
        let preset = fleet_preset();
        let status = apply_colour_preset(&camera, &ENTITIES, &preset);
        assert_eq!(status["matched"], false);
        assert!(status["refused"]["hue"].is_string(), "{status}");
        assert_eq!(status["read_back"]["hue"], -12);
        assert_eq!(status["read_back"]["saturation"], 64);
        assert_eq!(status["requested"]["saturation"], 60);
        assert_eq!(status["mismatched"], json!(["saturation", "hue"]));
        // The rest still went out.
        assert_eq!(status["read_back"]["gamma"], 100);
        let refusal = preset.refusal(&status).expect("refused");
        assert!(
            refusal.contains("saturation") && refusal.contains("hue"),
            "{refusal}"
        );
        let recorded = ColourPreset {
            refuse_on_mismatch: false,
            ..preset
        };
        let camera = Quirky {
            inner: Controls::default(),
            refuses: Some(PU_HUE_CONTROL),
            clamps: None,
            changes_gain_on_backlight: None,
        };
        camera.inner.state.borrow_mut().insert(
            (u16::from(PU_HUE_CONTROL) << 8, 0x0200),
            0_i16.to_le_bytes().to_vec(),
        );
        let status = apply_colour_preset(&camera, &ENTITIES, &recorded);
        assert_eq!(status["read_back"]["hue"], 0);
        assert_eq!(status["mismatched"], json!([]));
        assert!(
            status["refused"]["hue"]
                .as_str()
                .unwrap()
                .starts_with("SET_CUR:"),
            "{status}"
        );
        assert_eq!(status["matched"], false);
        assert_eq!(status["refuse_on_mismatch"], false);
        assert_eq!(recorded.refusal(&status), None);
    }

    #[test]
    fn a_camera_left_in_auto_exposure_reads_back_as_a_mismatch() {
        let stubborn = Controls {
            hold_auto: true,
            ..Controls::default()
        };
        let preset = fleet_preset();
        let status = apply_colour_preset(&stubborn, &ENTITIES, &preset);
        assert_eq!(status["read_back"]["auto_exposure"], 3, "aperture priority");
        assert_eq!(status["mismatched"], json!(["auto_exposure"]));
        assert!(preset.refusal(&status).is_some());
    }

    #[test]
    fn presets_are_validated_against_the_cameras_ranges() {
        let preset = fleet_preset();
        assert_eq!(preset.validate(), Ok(()));
        assert_eq!(
            preset.exposure_config(),
            ExposureConfig {
                start: Setting {
                    exposure_100us: 23,
                    gain: 0
                },
                min: Setting {
                    exposure_100us: 23,
                    gain: 0
                },
                max: Setting {
                    exposure_100us: 23,
                    gain: 0
                },
            }
        );
        let edges = ColourPreset {
            exposure_100us: 5000,
            gain: 100,
            wb_temperature: 6500,
            saturation: 128,
            contrast: 64,
            gamma: 72,
            brightness: -64,
            hue: 40,
            sharpness: 6,
            ..fleet_preset()
        };
        assert_eq!(edges.validate(), Ok(()));
        for (bad, field) in [
            (
                ColourPreset {
                    auto_exposure: 3,
                    ..fleet_preset()
                },
                "auto_exposure 3",
            ),
            (
                ColourPreset {
                    exposure_100us: 0,
                    ..fleet_preset()
                },
                "exposure 0",
            ),
            (
                ColourPreset {
                    exposure_100us: 5001,
                    ..fleet_preset()
                },
                "exposure 5001",
            ),
            (
                ColourPreset {
                    gain: 101,
                    ..fleet_preset()
                },
                "gain 101",
            ),
            (
                ColourPreset {
                    wb_temperature: 2799,
                    ..fleet_preset()
                },
                "wb_temperature 2799",
            ),
            (
                ColourPreset {
                    saturation: 129,
                    ..fleet_preset()
                },
                "saturation 129",
            ),
            (
                ColourPreset {
                    contrast: 65,
                    ..fleet_preset()
                },
                "contrast 65",
            ),
            (
                ColourPreset {
                    gamma: 71,
                    ..fleet_preset()
                },
                "gamma 71",
            ),
            (
                ColourPreset {
                    brightness: 65,
                    ..fleet_preset()
                },
                "brightness 65",
            ),
            (
                ColourPreset {
                    hue: -41,
                    ..fleet_preset()
                },
                "hue -41",
            ),
            (
                ColourPreset {
                    sharpness: 7,
                    ..fleet_preset()
                },
                "sharpness 7",
            ),
            (
                ColourPreset {
                    auto_wb: 2,
                    ..fleet_preset()
                },
                "auto_wb 2",
            ),
            (
                ColourPreset {
                    power_line_frequency: Some(3),
                    ..fleet_preset()
                },
                "power_line_frequency 3",
            ),
        ] {
            let error = bad.validate().unwrap_err();
            assert!(error.contains(field), "{field}: {error}");
        }
        let unnamed = ColourPreset {
            name: " ".to_owned(),
            ..fleet_preset()
        };
        assert!(unnamed.validate().is_err());
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
