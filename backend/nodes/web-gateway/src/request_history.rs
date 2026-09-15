use redis::AsyncCommands;
use serde_json::{Value, json};
use std::{
    collections::BTreeSet,
    sync::Mutex,
    time::{SystemTime, UNIX_EPOCH},
};
use tokio::sync::mpsc;

struct Update {
    input: String,
    value: Value,
}

/// Redis owns request records; this queue keeps its IO off Dora's event thread.
pub struct RequestHistory {
    client: redis::Client,
    session: String,
    ids: BTreeSet<String>,
    tx: mpsc::UnboundedSender<Update>,
    rx: Mutex<Option<mpsc::UnboundedReceiver<Update>>>,
}

impl Default for RequestHistory {
    fn default() -> Self {
        let (tx, rx) = mpsc::unbounded_channel();
        Self {
            client: redis::Client::open(
                std::env::var("REDIS_URL").unwrap_or_else(|_| "redis://redis:6379".into()),
            )
            .expect("valid REDIS_URL"),
            session: format!(
                "{}-{}",
                std::process::id(),
                SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .expect("system clock")
                    .as_nanos()
            ),
            ids: BTreeSet::new(),
            tx,
            rx: Mutex::new(Some(rx)),
        }
    }
}

impl RequestHistory {
    pub fn start(&self) -> tokio::task::JoinHandle<()> {
        let mut rx = self
            .rx
            .lock()
            .expect("history queue")
            .take()
            .expect("history starts once");
        let client = self.client.clone();
        let session = self.session.clone();
        tokio::spawn(async move {
            while let Some(update) = rx.recv().await {
                loop {
                    match persist(&client, &session, &update).await {
                        Ok(()) => break,
                        Err(error) => {
                            eprintln!("Redis 请求记录写入失败，将重试同一记录：{error}");
                            tokio::time::sleep(std::time::Duration::from_secs(1)).await;
                        }
                    }
                }
            }
        })
    }

    pub fn begin(&mut self, output: &str, body: &Value) {
        if output.ends_with("asset_request") || output.ends_with("snapshot") {
            return;
        }
        if let Some(id) = body["request_id"].as_str().filter(|id| !id.is_empty()) {
            self.ids.insert(id.into());
        }
    }

    pub fn observe(&mut self, input: &str, value: &Value) {
        let id = value["request_id"]
            .as_str()
            .or_else(|| value["task_request_id"].as_str());
        if id.is_some_and(|id| self.ids.contains(id)) {
            let id = id.expect("registered request");
            let Some(state) = request_state(input, value, id) else {
                return;
            };
            // Queue the terminal result once. The writer retains it through
            // Redis retries; periodic node snapshots must not grow this queue.
            if terminal(state) {
                self.ids.remove(id);
            }
            let _ = self.tx.send(Update {
                input: input.into(),
                value: value.clone(),
            });
        }
    }

    pub fn reader(&self) -> (redis::Client, String) {
        (self.client.clone(), self.session.clone())
    }
}

fn key(id: &str) -> String {
    format!("robot-arm:request:{id}")
}

pub async fn reserve(
    client: redis::Client,
    session: String,
    output: &str,
    body: &Value,
) -> eyre::Result<bool> {
    if output.ends_with("asset_request") {
        return Ok(true);
    }
    let id = body["request_id"].as_str().unwrap_or_default();
    eyre::ensure!(!id.is_empty(), "request_id 不能为空");
    let mut connection = client.get_multiplexed_async_connection().await?;
    let record = json!({
        "schema_version": robot_arm_messages::SCHEMA_VERSION, "request_id": id,
        "session_id": session, "operation": output, "state": "accepted", "terminal": false,
        "value": null, "original_error": null,
        "updated_at_ms": SystemTime::now().duration_since(UNIX_EPOCH)?.as_millis() as u64,
    });
    Ok(connection
        .set_nx(key(id), serde_json::to_string(&record)?)
        .await?)
}

pub async fn read(client: redis::Client, session: String, id: &str) -> eyre::Result<Option<Value>> {
    let mut connection = client.get_multiplexed_async_connection().await?;
    let value: Option<String> = connection.get(key(id)).await?;
    value
        .map(|raw| {
            let mut value: Value = serde_json::from_str(&raw)?;
            if value["session_id"] != session && value["terminal"] != true {
                value["state"] = json!("unknown");
                value["original_error"] =
                    json!("服务已重启，未确认该请求的最终执行结果；不会自动重放");
            }
            Ok(value)
        })
        .transpose()
}

async fn persist(client: &redis::Client, session: &str, update: &Update) -> eyre::Result<()> {
    let mut connection = client.get_multiplexed_async_connection().await?;
    let Update { input, value } = update;
    let id = value["request_id"]
        .as_str()
        .or_else(|| value["task_request_id"].as_str())
        .expect("registered request");
    let Some(raw): Option<String> = connection.get(key(id)).await? else {
        return Ok(());
    };
    let mut record: Value = serde_json::from_str(&raw)?;
    if record["session_id"] != session || !update_record(&mut record, input, value) {
        return Ok(());
    }
    record["updated_at_ms"] =
        json!(SystemTime::now().duration_since(UNIX_EPOCH)?.as_millis() as u64);
    let _: () = connection
        .set(key(id), serde_json::to_string(&record)?)
        .await?;
    Ok(())
}

fn terminal(state: &str) -> bool {
    matches!(state, "succeeded" | "failed" | "cancelled")
}

fn request_state<'a>(input: &str, value: &'a Value, id: &str) -> Option<&'a str> {
    let payload = value.get("value").filter(|v| !v.is_null()).unwrap_or(value);
    let error = value["original_error"].as_str();
    error
        .map(|_| "failed")
        .or_else(|| payload["state"].as_str())
        .or_else(|| {
            (payload["task_request_id"].as_str() == Some(id))
                .then(|| payload["task_state"].as_str())
                .flatten()
        })
        .or_else(|| input.ends_with("request_result").then_some("succeeded"))
}

fn update_record(record: &mut Value, input: &str, value: &Value) -> bool {
    let payload = value.get("value").filter(|v| !v.is_null()).unwrap_or(value);
    let error = value["original_error"].as_str();
    let Some(state) = request_state(
        input,
        value,
        record["request_id"].as_str().unwrap_or_default(),
    ) else {
        return false;
    };
    if record["terminal"] == true {
        return false;
    }
    if record["state"] == state && record["value"] == *payload {
        return false;
    }
    record["state"] = json!(state);
    record["terminal"] = json!(terminal(state));
    record["value"] = payload.clone();
    record["original_error"] = json!(error.or_else(|| {
        (state == "failed")
            .then(|| payload["original_error"].as_str())
            .flatten()
    }));
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    #[tokio::test]
    async fn writer_finishes_when_last_history_owner_is_dropped() {
        let history = std::sync::Arc::new(RequestHistory::default());
        let http_owner = history.clone();
        let worker = history.start();
        drop(history);
        assert!(!worker.is_finished());
        drop(http_owner);
        tokio::time::timeout(std::time::Duration::from_secs(1), worker)
            .await
            .expect("closed history queue must let the writer finish")
            .expect("history writer must not panic");
    }

    #[test]
    fn completed_requests_leave_the_observation_set() {
        let mut history = RequestHistory::default();
        history.begin("motion_request", &json!({"request_id":"a"}));
        history.observe(
            "motion_status",
            &json!({"request_id":"a","state":"executing"}),
        );
        assert!(history.ids.contains("a"));
        history.observe(
            "motion_status",
            &json!({"request_id":"a","state":"succeeded"}),
        );
        assert!(!history.ids.contains("a"));
        history.observe(
            "motion_status",
            &json!({"request_id":"a","state":"succeeded"}),
        );
        let mut receiver = history.rx.lock().unwrap().take().unwrap();
        assert!(receiver.try_recv().is_ok());
        assert!(receiver.try_recv().is_ok());
        assert!(receiver.try_recv().is_err());
    }
    #[test]
    fn acceptance_is_not_completion_and_terminal_is_not_overwritten() {
        let mut record = json!({"request_id":"a","terminal":false});
        update_record(
            &mut record,
            "manipulation_request_result",
            &json!({"request_id":"a","value":{"state":"planning"}}),
        );
        assert_eq!(record["terminal"], false);
        update_record(
            &mut record,
            "manipulation_state",
            &json!({"request_id":"a","state":"cancelled"}),
        );
        update_record(
            &mut record,
            "manipulation_state",
            &json!({"request_id":"a","state":"executing"}),
        );
        assert_eq!(record["state"], "cancelled");
    }
    #[test]
    fn configuration_ack_does_not_inherit_another_tasks_error_state() {
        let mut record = json!({"request_id":"a","terminal":false});
        update_record(
            &mut record,
            "perception_request_result",
            &json!({"request_id":"a","value":{"task_request_id":"old","task_state":"executing"}}),
        );
        assert_eq!(record["state"], "succeeded");
    }

    #[tokio::test]
    #[ignore = "requires REDIS_TEST_URL pointing at the Compose Redis service"]
    async fn redis_reserves_once_and_preserves_results_across_sessions() -> eyre::Result<()> {
        let client = redis::Client::open(std::env::var("REDIS_TEST_URL")?)?;
        let id = format!(
            "test-{}",
            SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos()
        );
        let body = json!({"request_id": id});
        assert!(reserve(client.clone(), "first".into(), "motion_request", &body).await?);
        assert!(!reserve(client.clone(), "second".into(), "motion_request", &body).await?);
        assert_eq!(
            read(client.clone(), "first".into(), &id).await?.unwrap()["state"],
            "accepted"
        );
        assert_eq!(
            read(client.clone(), "second".into(), &id).await?.unwrap()["state"],
            "unknown"
        );
        persist(&client, "first", &Update {
            input: "motion_request_result".into(),
            value: json!({"request_id": id,"value":{"state":"failed","original_error":"controller failure"}}),
        }).await?;
        let result = read(client.clone(), "second".into(), &id).await?.unwrap();
        assert_eq!(result["state"], "failed");
        assert_eq!(result["original_error"], "controller failure");
        assert_eq!(result["terminal"], true);
        let mut connection = client.get_multiplexed_async_connection().await?;
        let _: () = connection.del(key(&id)).await?;
        assert!(read(client, "first".into(), &id).await?.is_none());
        Ok(())
    }
}
