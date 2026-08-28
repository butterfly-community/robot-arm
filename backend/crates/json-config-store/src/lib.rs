use std::{fs, io::ErrorKind, path::Path};

use eyre::{Context, Result};
use serde::{Serialize, de::DeserializeOwned};

pub fn load_or_default<T>(path: &Path) -> Result<T>
where
    T: Default + DeserializeOwned,
{
    let bytes = match fs::read(path) {
        Ok(bytes) => bytes,
        Err(error) if error.kind() == ErrorKind::NotFound => return Ok(T::default()),
        Err(error) => return Err(error).wrap_err_with(|| format!("读取 {}", path.display())),
    };
    serde_json::from_slice(&bytes).wrap_err_with(|| format!("解析 {}", path.display()))
}

pub fn save<T: Serialize>(path: &Path, config: &T) -> Result<()> {
    let bytes = serde_json::to_vec_pretty(config)?;
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(path, bytes).wrap_err_with(|| format!("写入 {}", path.display()))
}

#[cfg(test)]
mod tests {
    use std::time::{SystemTime, UNIX_EPOCH};

    use serde::{Deserialize, Serialize};

    use super::*;

    #[derive(Debug, Default, PartialEq, Serialize, Deserialize)]
    struct Fixture {
        value: u64,
    }

    fn path(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!(
            "json-config-store-{name}-{}-{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ))
    }

    #[test]
    fn missing_file_uses_default() {
        assert_eq!(
            load_or_default::<Fixture>(&path("missing")).unwrap(),
            Fixture::default()
        );
    }

    #[test]
    fn round_trip_preserves_config() {
        let path = path("round-trip");
        let expected = Fixture { value: 7 };
        save(&path, &expected).unwrap();
        assert_eq!(load_or_default::<Fixture>(&path).unwrap(), expected);
        fs::remove_file(path).unwrap();
    }

    #[test]
    fn malformed_existing_file_is_an_error() {
        let path = path("malformed");
        fs::write(&path, b"{").unwrap();
        assert!(load_or_default::<Fixture>(&path).is_err());
        fs::remove_file(path).unwrap();
    }
}
