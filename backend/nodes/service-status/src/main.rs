use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
    time::{SystemTime, UNIX_EPOCH},
};

use dora_node_api::{DoraNode, Event, MetadataParameters, dora_core::config::DataId};
use eyre::{Context, Result};
use robot_arm_messages::{
    SCHEMA_VERSION, ServiceReadiness, ServiceState, SystemReadiness, from_arrow, to_arrow,
};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct Config {
    services: BTreeMap<String, Vec<String>>,
}

fn main() -> Result<()> {
    let config_path = std::env::var_os("SERVICE_STATUS_CONFIG")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/config/service-status.json"));
    let config = load_config(&config_path)?;
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut states = BTreeMap::new();
    publish(&mut node, &evaluate(&config, &states, now_ns()))?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } => {
                let state: ServiceState = from_arrow(data.as_array())
                    .with_context(|| format!("decode service report {id}"))?;
                states.insert(id.to_string(), state);
                publish(&mut node, &evaluate(&config, &states, now_ns()))?;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}

fn load_config(path: &Path) -> Result<Config> {
    let bytes = fs::read(path).with_context(|| format!("read {}", path.display()))?;
    let config: Config =
        serde_json::from_slice(&bytes).with_context(|| format!("decode {}", path.display()))?;
    for (service, dependencies) in &config.services {
        for dependency in dependencies {
            eyre::ensure!(
                config.services.contains_key(dependency),
                "service {service} depends on unknown service {dependency}"
            );
        }
    }
    Ok(config)
}

fn evaluate(
    config: &Config,
    states: &BTreeMap<String, ServiceState>,
    updated_at_ns: i64,
) -> SystemReadiness {
    let reported = config
        .services
        .keys()
        .map(|service| {
            let ready = states.get(service).is_some_and(|state| state.running);
            (service.clone(), ready)
        })
        .collect::<BTreeMap<_, _>>();
    let mut effective = reported.clone();
    for _ in 0..config.services.len() {
        let next = config
            .services
            .iter()
            .map(|(service, dependencies)| {
                (
                    service.clone(),
                    reported[service]
                        && dependencies.iter().all(|dependency| effective[dependency]),
                )
            })
            .collect::<BTreeMap<_, _>>();
        if next == effective {
            break;
        }
        effective = next;
    }
    let services = config
        .services
        .iter()
        .map(|(service_id, dependencies)| {
            let reported_ready = reported[service_id];
            let blocked_by = dependencies
                .iter()
                .filter(|dependency| !effective[*dependency])
                .cloned()
                .collect::<Vec<_>>();
            ServiceReadiness {
                service_id: service_id.clone(),
                reported_ready,
                ready: effective[service_id],
                blocked_by,
                state: states.get(service_id).cloned(),
            }
        })
        .collect::<Vec<_>>();
    SystemReadiness {
        schema_version: SCHEMA_VERSION,
        ready: services.iter().all(|service| service.ready),
        services,
        updated_at_ns,
    }
}

fn publish(node: &mut DoraNode, value: &SystemReadiness) -> Result<()> {
    node.send_output(
        DataId::from("system_readiness".to_owned()),
        MetadataParameters::default(),
        to_arrow(value)?,
    )?;
    Ok(())
}

fn now_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_nanos() as i64)
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state(output: bool) -> ServiceState {
        ServiceState {
            schema_version: SCHEMA_VERSION,
            running: true,
            has_output: output,
            ..ServiceState::default()
        }
    }

    #[test]
    fn missing_and_unready_dependencies_are_reported_without_timeout_logic() {
        let config = Config {
            services: BTreeMap::from([
                ("source".into(), vec![]),
                ("consumer".into(), vec!["source".into()]),
            ]),
        };
        let states = BTreeMap::from([("consumer".into(), state(true))]);
        let readiness = evaluate(&config, &states, 7);
        assert!(!readiness.ready);
        assert_eq!(readiness.services[0].service_id, "consumer");
        assert_eq!(readiness.services[0].blocked_by, ["source"]);
        assert!(!readiness.services[1].reported_ready);
    }

    #[test]
    fn all_running_dependencies_become_ready_without_domain_specific_output_rules() {
        let config = Config {
            services: BTreeMap::from([
                ("source".into(), vec![]),
                ("consumer".into(), vec!["source".into()]),
            ]),
        };
        let states = BTreeMap::from([
            ("source".into(), state(false)),
            ("consumer".into(), state(false)),
        ]);
        let readiness = evaluate(&config, &states, 9);
        assert!(readiness.ready);
        assert!(readiness.services.iter().all(|service| service.ready));
    }
}
