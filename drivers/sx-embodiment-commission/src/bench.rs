//! What every bench tool opens a body from: the robot instance's drive, the station's
//! commissioning, any serial unit calibrations, the provisional ports the run takes on, and
//! joints left out of service for this run. A tool parses these flags with [`BodyFlags::take`],
//! then opens the body to drive ([`BodyFlags::open`]) or observes it without writing
//! ([`BodyFlags::observe`]), through the embodiment layer and nothing else.
//!
//! ```text
//! --drive <drive.json> --commissioning <commissioning.json>
//! [--calibration <port>=<lerobot.json>]... [--allow-provisional <port>]... [--exclude <joint,...>]
//! ```

use sx_embodiment_bodies::system::SystemPorts;
use sx_embodiment_bodies::{
    Body, Commissioning, Observer, OpenOptions, ServoCalibrations, observe, open_with,
};
use sx_embodiment_drivers::InstanceDrive;

/// The body a bench tool works on, as its flags name it.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct BodyFlags {
    pub drive: String,
    pub commissioning: String,
    pub calibrations: Vec<(String, String)>,
    pub provisional: Vec<String>,
    pub exclude: Vec<String>,
}

impl BodyFlags {
    /// Take `arg` if it is one of these flags, reading its value from `value`: `Ok(true)` when
    /// taken, `Ok(false)` when it is another tool's.
    ///
    /// # Errors
    ///
    /// A flag of these whose value is missing or malformed.
    pub fn take(
        &mut self,
        arg: &str,
        value: &mut dyn FnMut() -> Result<String, String>,
    ) -> Result<bool, String> {
        match arg {
            "--drive" => self.drive = value()?,
            "--commissioning" => self.commissioning = value()?,
            "--calibration" => {
                let entry = value()?;
                let (port, file) = entry
                    .split_once('=')
                    .ok_or("--calibration is <port>=<lerobot.json>")?;
                self.calibrations.push((port.to_owned(), file.to_owned()));
            }
            "--allow-provisional" => self.provisional.push(value()?),
            "--exclude" => self.exclude.extend(
                value()?
                    .split(',')
                    .filter(|joint| !joint.is_empty())
                    .map(str::to_owned),
            ),
            _ => return Ok(false),
        }
        Ok(true)
    }

    /// The drive, commissioning (this run's exclusions out of service beside its own) and
    /// calibrations these flags name, read and parsed.
    ///
    /// # Errors
    ///
    /// A file missing, unreadable or refused by its parser, and a drive naming no instance.
    pub fn inputs(&self) -> Result<(InstanceDrive, Commissioning, ServoCalibrations), String> {
        if self.drive.is_empty() || self.commissioning.is_empty() {
            return Err("--drive and --commissioning are required".into());
        }
        let document = read(&self.drive)?;
        let instance = serde_json::from_str::<serde_json::Value>(&document)
            .map_err(|error| format!("{}: {error}", self.drive))?["instance"]
            .as_str()
            .ok_or("the drive names no instance")?
            .to_owned();
        let drive =
            InstanceDrive::parse(&document, &instance).map_err(|error| error.to_string())?;
        let mut commissioning = Commissioning::parse(&read(&self.commissioning)?, &instance)
            .map_err(|error| error.to_string())?;
        commissioning
            .out_of_service
            .extend(self.exclude.iter().cloned());
        let mut calibrations = ServoCalibrations::new();
        for (port, file) in &self.calibrations {
            calibrations = calibrations
                .with_lerobot(port, &read(file)?)
                .map_err(|error| error.to_string())?;
        }
        Ok((drive, commissioning, calibrations))
    }

    /// Open the body to drive, as a station opens it.
    ///
    /// # Errors
    ///
    /// As [`BodyFlags::inputs`], and whatever the body's opener refuses.
    pub fn open(&self) -> Result<Body, String> {
        let (drive, commissioning, calibrations) = self.inputs()?;
        let provisional: Vec<&str> = self.provisional.iter().map(String::as_str).collect();
        open_with(
            &drive,
            &commissioning,
            &calibrations,
            OpenOptions {
                allow_provisional: &provisional,
            },
            &mut SystemPorts::system(),
        )
        .map_err(|error| error.to_string())
    }

    /// Observe the body without writing anything to it.
    ///
    /// # Errors
    ///
    /// As [`BodyFlags::inputs`], and whatever the body's observer refuses.
    pub fn observe(&self) -> Result<Observer, String> {
        let (drive, commissioning, calibrations) = self.inputs()?;
        let provisional: Vec<&str> = self.provisional.iter().map(String::as_str).collect();
        observe(
            &drive,
            &commissioning,
            &calibrations,
            OpenOptions {
                allow_provisional: &provisional,
            },
            &mut SystemPorts::system(),
        )
        .map_err(|error| error.to_string())
    }
}

fn read(path: &str) -> Result<String, String> {
    std::fs::read_to_string(path).map_err(|error| format!("{path}: {error}"))
}
