//! Persistent per-source gyroscope bias values.

use anyhow::{Context, Result, bail};
use fusion_ahrs::OffsetSettings;
use serde::{Deserialize, Serialize};
use std::{fs, io::ErrorKind, path::Path};

pub const SOURCE_COUNT: usize = 3;
const SCHEMA_VERSION: u32 = 1;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct GyroBiasStore {
    schema_version: u32,
    bias_dps: [Option<[f32; 3]>; SOURCE_COUNT],
}

impl Default for GyroBiasStore {
    fn default() -> Self {
        Self {
            schema_version: SCHEMA_VERSION,
            bias_dps: [None; SOURCE_COUNT],
        }
    }
}

impl GyroBiasStore {
    pub fn load(path: &Path) -> Result<Self> {
        let bytes = match fs::read(path) {
            Ok(bytes) => bytes,
            Err(error) if error.kind() == ErrorKind::NotFound => return Ok(Self::default()),
            Err(error) => {
                return Err(error).with_context(|| format!("failed to read {}", path.display()));
            }
        };
        let store: Self = serde_json::from_slice(&bytes)
            .with_context(|| format!("invalid gyroscope bias file {}", path.display()))?;
        store.validate()?;
        Ok(store)
    }

    pub fn bias(&self, source_id: usize) -> Option<[f32; 3]> {
        self.bias_dps.get(source_id).copied().flatten()
    }

    pub fn set_bias(&mut self, source_id: usize, bias_dps: [f32; 3]) -> Result<()> {
        if source_id >= SOURCE_COUNT {
            bail!("invalid gyroscope source id {source_id}");
        }
        validate_bias(bias_dps)?;
        self.bias_dps[source_id] = Some(bias_dps);
        Ok(())
    }

    pub fn save(&self, path: &Path) -> Result<()> {
        self.validate()?;
        let parent = path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
            .context("gyroscope bias file has no parent directory")?;
        fs::create_dir_all(parent)
            .with_context(|| format!("failed to create {}", parent.display()))?;
        let temporary = path.with_extension("json.tmp");
        let mut bytes = serde_json::to_vec_pretty(self)?;
        bytes.push(b'\n');
        fs::write(&temporary, bytes)
            .with_context(|| format!("failed to write {}", temporary.display()))?;
        fs::rename(&temporary, path).with_context(|| {
            format!(
                "failed to replace {} with {}",
                path.display(),
                temporary.display()
            )
        })?;
        Ok(())
    }

    fn validate(&self) -> Result<()> {
        if self.schema_version != SCHEMA_VERSION {
            bail!(
                "unsupported gyroscope bias schema version {}",
                self.schema_version
            );
        }
        for bias in self.bias_dps.into_iter().flatten() {
            validate_bias(bias)?;
        }
        Ok(())
    }
}

fn validate_bias(bias_dps: [f32; 3]) -> Result<()> {
    let threshold = OffsetSettings::default().threshold;
    if bias_dps
        .into_iter()
        .any(|value| !value.is_finite() || value.abs() > threshold)
    {
        bail!("gyroscope bias must be finite and within Fusion's stationary threshold");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn temporary_path() -> std::path::PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        std::env::temp_dir()
            .join(format!("nolo-gyro-bias-{}-{nonce}", std::process::id()))
            .join("gyro-bias-v1.json")
    }

    #[test]
    fn missing_file_starts_without_a_saved_bias() {
        let path = temporary_path();
        let store = GyroBiasStore::load(&path).unwrap();
        assert!((0..SOURCE_COUNT).all(|source_id| store.bias(source_id).is_none()));
    }

    #[test]
    fn saved_bias_round_trips_per_source() {
        let path = temporary_path();
        let mut store = GyroBiasStore::default();
        store.set_bias(0, [0.1, -0.2, 0.3]).unwrap();
        store.set_bias(2, [-0.4, 0.5, -0.6]).unwrap();
        store.save(&path).unwrap();

        let loaded = GyroBiasStore::load(&path).unwrap();
        assert_eq!(loaded.bias(0), Some([0.1, -0.2, 0.3]));
        assert_eq!(loaded.bias(1), None);
        assert_eq!(loaded.bias(2), Some([-0.4, 0.5, -0.6]));

        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }

    #[test]
    fn non_finite_or_implausible_bias_is_rejected() {
        let mut store = GyroBiasStore::default();
        assert!(store.set_bias(0, [f32::NAN, 0.0, 0.0]).is_err());
        let threshold = OffsetSettings::default().threshold;
        assert!(store.set_bias(0, [threshold + 0.1, 0.0, 0.0]).is_err());
    }
}
