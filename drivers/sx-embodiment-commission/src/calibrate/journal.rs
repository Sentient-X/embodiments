//! What a calibration run changed, on disk before the bus sees the change.
//!
//! One journal per bus, named for its device node, in the journal directory
//! (`$XDG_STATE_HOME/sx-commission/calibrate`, else `~/.local/state/sx-commission/calibrate`).
//! It is created when a session starts, refused when one is already there, rewritten (to a
//! sibling file, synced, renamed over it) before every change it records, and removed only
//! when every servo it names is proven limp and every register it saved is read back restored.

use std::fs::{self, File, OpenOptions};
use std::io::{self, Write as _};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use super::CalibrateError;

/// The journal's schema.
pub const JOURNAL_SCHEMA: &str = "sx.feetech-calibrate-journal/v1";

/// One register's value before the run first changed it.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SavedRegister {
    pub id: u8,
    pub address: u8,
    /// 1 or 2 bytes.
    pub width: u8,
    pub value: u16,
}

/// What a run has done that is not yet undone.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct JournalRecord {
    pub schema: String,
    pub bus: String,
    pub baud: u32,
    pub pid: u32,
    /// Every servo the run may have powered, in the order enabled.
    pub powered: Vec<u8>,
    /// Every register the run changed, with its value before, in the order changed.
    pub saved: Vec<SavedRegister>,
}

/// The run's journal file.
#[derive(Debug)]
pub struct Journal {
    path: PathBuf,
    pub(crate) record: JournalRecord,
}

fn journal_error(path: &Path, source: io::Error) -> CalibrateError {
    CalibrateError::Journal {
        path: path.to_owned(),
        source,
    }
}

impl Journal {
    /// The default journal directory.
    #[must_use]
    pub fn default_dir() -> PathBuf {
        let state = std::env::var_os("XDG_STATE_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|home| Path::new(&home).join(".local/state")))
            .unwrap_or_else(std::env::temp_dir);
        state.join("sx-commission").join("calibrate")
    }

    /// The journal for `bus` in `dir`: its device node's name, every other byte an underscore.
    #[must_use]
    pub fn path_for(dir: &Path, bus: &str) -> PathBuf {
        let name: String = Path::new(bus)
            .file_name()
            .map_or_else(|| bus.into(), |name| name.to_string_lossy())
            .chars()
            .map(|c| {
                if c.is_ascii_alphanumeric() || c == '.' || c == '-' {
                    c
                } else {
                    '_'
                }
            })
            .collect();
        dir.join(format!("{name}.json"))
    }

    /// Start `bus`'s journal in `dir`.
    ///
    /// # Errors
    ///
    /// [`CalibrateError::Unrestored`] when an earlier run's journal is there; the file
    /// system's failure otherwise.
    pub fn create(dir: &Path, bus: &str, baud: u32) -> Result<Self, CalibrateError> {
        fs::create_dir_all(dir).map_err(|e| journal_error(dir, e))?;
        let path = Self::path_for(dir, bus);
        let journal = Self {
            record: JournalRecord {
                schema: JOURNAL_SCHEMA.to_owned(),
                bus: bus.to_owned(),
                baud,
                pid: std::process::id(),
                powered: Vec::new(),
                saved: Vec::new(),
            },
            path,
        };
        let mut file = match OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&journal.path)
        {
            Ok(file) => file,
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {
                return Err(CalibrateError::Unrestored { path: journal.path });
            }
            Err(e) => return Err(journal_error(&journal.path, e)),
        };
        file.write_all(&journal.bytes())
            .and_then(|()| file.sync_all())
            .map_err(|e| journal_error(&journal.path, e))?;
        Ok(journal)
    }

    /// An earlier run's journal.
    ///
    /// # Errors
    ///
    /// The file system's failure, or a file that is not a journal.
    pub fn load(path: &Path) -> Result<Self, CalibrateError> {
        let text = fs::read_to_string(path).map_err(|e| journal_error(path, e))?;
        let record: JournalRecord = serde_json::from_str(&text)
            .map_err(|e| journal_error(path, io::Error::new(io::ErrorKind::InvalidData, e)))?;
        if record.schema != JOURNAL_SCHEMA {
            return Err(journal_error(
                path,
                io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("schema {} is not {JOURNAL_SCHEMA}", record.schema),
                ),
            ));
        }
        Ok(Self {
            path: path.to_owned(),
            record,
        })
    }

    #[must_use]
    pub fn path(&self) -> &Path {
        &self.path
    }

    #[must_use]
    pub const fn record(&self) -> &JournalRecord {
        &self.record
    }

    fn bytes(&self) -> Vec<u8> {
        let mut text = serde_json::to_string_pretty(&self.record).unwrap_or_default();
        text.push('\n');
        text.into_bytes()
    }

    /// Write the record to a sibling file, sync it and rename it over the journal.
    pub(crate) fn persist(&self) -> Result<(), CalibrateError> {
        let staging = self.path.with_extension("json.next");
        let write = || -> io::Result<()> {
            let mut file = File::create(&staging)?;
            file.write_all(&self.bytes())?;
            file.sync_all()?;
            fs::rename(&staging, &self.path)?;
            if let Some(dir) = self.path.parent()
                && let Ok(dir) = File::open(dir)
            {
                // A directory sync makes the rename durable where the platform supports it.
                let _ = dir.sync_all();
            }
            Ok(())
        };
        write().map_err(|e| journal_error(&self.path, e))
    }

    /// Remove the journal: everything it named is undone.
    pub(crate) fn discard(&self) -> Result<(), CalibrateError> {
        fs::remove_file(&self.path).map_err(|e| journal_error(&self.path, e))
    }
}
