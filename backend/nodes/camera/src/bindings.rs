//! Role bindings share the capture worker and RGB contract with depth preview.
use crate::{
    capture_worker::{self, Command, Completion, Output},
    video_server::VideoStreams,
};
use eyre::{Result, bail, eyre};
use robot_arm_messages::{
    CameraBindingState, CameraRequest, CameraRole, CameraSourceInfo, CameraStreamKind,
    ImageFrameInfo, RequestAction,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct SavedBinding {
    pub source: CameraSourceInfo,
    pub color_profile_key: String,
    #[serde(default = "enabled_by_default")]
    pub enabled: bool,
}

fn enabled_by_default() -> bool {
    true
}

struct Slot {
    channels: capture_worker::Channels,
    binding: Option<SavedBinding>,
    pending: Option<(String, Option<SavedBinding>)>,
    streaming: bool,
    error: Option<String>,
}

pub(crate) struct Bindings {
    slots: BTreeMap<CameraRole, Slot>,
}

impl Bindings {
    pub fn new(
        runtime: &tokio::runtime::Runtime,
        saved: &BTreeMap<CameraRole, SavedBinding>,
    ) -> Self {
        let mut slots = BTreeMap::new();
        for role in [CameraRole::External, CameraRole::Wrist] {
            let channels = capture_worker::spawn(runtime);
            let binding = saved.get(&role).cloned();
            let mut error = None;
            let mut pending = None;
            if let Some(b) = binding.as_ref().filter(|b| b.enabled) {
                if let Some(color) = b
                    .source
                    .profiles
                    .iter()
                    .find(|p| p.key == b.color_profile_key)
                {
                    let request_id = format!("camera-restore-{role:?}");
                    if let Err(e) = channels.commands.try_send(Command::OpenColor {
                        request_id: request_id.clone(),
                        action: RequestAction::Connect,
                        source_id: b.source.source_id.clone(),
                        color: color.clone(),
                    }) {
                        error = Some(e.to_string());
                    } else {
                        pending = Some((request_id, Some(b.clone())));
                    }
                } else {
                    error = Some("保存的流配置不存在，请重新选择并保存".into());
                }
            }
            slots.insert(
                role,
                Slot {
                    channels,
                    binding,
                    pending,
                    streaming: false,
                    error,
                },
            );
        }
        Self { slots }
    }

    pub fn videos(&self) -> VideoStreams {
        self.slots
            .iter()
            .map(|(role, slot)| (*role, slot.channels.video.clone()))
            .collect()
    }

    pub fn saved(&self) -> BTreeMap<CameraRole, SavedBinding> {
        self.slots
            .iter()
            .filter_map(|(r, s)| s.binding.clone().map(|b| (*r, b)))
            .collect()
    }

    pub fn owns_source(&self, source_id: &str, except: CameraRole) -> bool {
        self.slots.iter().any(|(role, slot)| {
            *role != except
                && (slot
                    .binding
                    .as_ref()
                    .is_some_and(|b| b.source.source_id == source_id)
                    || slot
                        .pending
                        .as_ref()
                        .and_then(|p| p.1.as_ref())
                        .is_some_and(|b| b.source.source_id == source_id))
        })
    }

    pub fn apply(
        &mut self,
        request: &CameraRequest,
        sources: &[CameraSourceInfo],
        depth_source: Option<&str>,
    ) -> Result<bool> {
        let mut request = request.clone();
        if request.action == RequestAction::Connect {
            let saved = self
                .slots
                .get(&request.role)
                .and_then(|s| s.binding.as_ref())
                .ok_or_else(|| eyre!("请先保存摄像头绑定"))?;
            request.source_id = Some(saved.source.source_id.clone());
            request.color_profile_key = Some(saved.color_profile_key.clone());
        }
        if matches!(
            request.action,
            RequestAction::Apply | RequestAction::Select | RequestAction::Connect
        ) {
            let source_id = request
                .source_id
                .as_deref()
                .ok_or_else(|| eyre!("请选择摄像头"))?;
            if depth_source == Some(source_id) || self.owns_source(source_id, request.role) {
                bail!("该相机已绑定其他角色，请先解除原绑定；不会重复打开同一设备");
            }
            let source = sources
                .iter()
                .find(|s| s.source_id == source_id && s.available)
                .ok_or_else(|| eyre!("相机未连接，请刷新设备列表；已保存绑定仍保留"))?;
            let color = source
                .profiles
                .iter()
                .find(|p| {
                    p.stream == CameraStreamKind::Color
                        && p.available
                        && request
                            .color_profile_key
                            .as_deref()
                            .is_none_or(|key| key == p.key)
                })
                .ok_or_else(|| eyre!("请选择相机支持的彩色流配置"))?;
            let slot = self
                .slots
                .get_mut(&request.role)
                .ok_or_else(|| eyre!("无效相机角色"))?;
            if slot.pending.is_some() {
                bail!("该相机配置正在执行，请等待结果");
            }
            let binding = SavedBinding {
                source: source.clone(),
                color_profile_key: color.key.clone(),
                enabled: true,
            };
            slot.channels.commands.try_send(Command::OpenColor {
                request_id: request.request_id.clone(),
                action: request.action,
                source_id: source_id.into(),
                color: color.clone(),
            })?;
            slot.pending = Some((request.request_id.clone(), Some(binding)));
            slot.streaming = false;
            slot.error = None;
            return Ok(true);
        }
        let slot = self
            .slots
            .get_mut(&request.role)
            .ok_or_else(|| eyre!("无效相机角色"))?;
        if slot.pending.is_some() {
            bail!("该相机配置正在执行，请等待结果");
        }
        match request.action {
            RequestAction::Disconnect | RequestAction::Unselect | RequestAction::Cancel => {
                let binding = if request.action == RequestAction::Unselect {
                    None
                } else {
                    slot.binding.clone().map(|mut b| {
                        b.enabled = false;
                        b
                    })
                };
                slot.channels.commands.try_send(Command::CloseColor {
                    request_id: request.request_id.clone(),
                    action: request.action,
                })?;
                slot.pending = Some((request.request_id.clone(), binding));
                slot.streaming = false;
                slot.error = None;
                Ok(true)
            }
            _ => bail!("此操作不适用于普通摄像头绑定"),
        }
    }

    pub fn tick(&mut self) -> Vec<(String, RequestAction, Option<String>)> {
        let mut results = vec![];
        for slot in self.slots.values_mut() {
            if slot.channels.commands.is_closed() {
                slot.streaming = false;
                slot.error = Some("相机采集线程已退出，当前没有图像信号".into());
                if let Some((id, _)) = slot.pending.take() {
                    results.push((id, RequestAction::Apply, slot.error.clone()));
                }
            }
            while let Ok(completion) = slot.channels.completions.try_recv() {
                if let Completion::Closed { request_id, action } = &completion {
                    if slot.pending.as_ref().is_some_and(|p| p.0 == *request_id) {
                        slot.binding = slot.pending.take().unwrap().1;
                        slot.streaming = false;
                        slot.error = None;
                        results.push((request_id.clone(), *action, None));
                    }
                    continue;
                }
                if let Completion::Opened {
                    request_id,
                    action,
                    result,
                    ..
                } = completion
                {
                    if slot.pending.as_ref().is_some_and(|p| p.0 == request_id) {
                        let (_, binding) = slot.pending.take().unwrap();
                        slot.streaming = result.is_ok();
                        if result.is_ok() {
                            slot.binding = binding;
                        }
                        slot.error = result.err();
                        results.push((request_id, action, slot.error.clone()));
                    }
                }
            }
            if slot.channels.outputs.has_changed().unwrap_or(false)
                && let Some(Output::Failed(error)) =
                    slot.channels.outputs.borrow_and_update().clone()
            {
                slot.streaming = false;
                slot.error = Some(error);
            }
        }
        results
    }

    pub fn states(&self) -> Vec<CameraBindingState> {
        self.slots
            .iter()
            .filter_map(|(role, slot)| {
                let binding = slot.binding.as_ref()?;
                let frame = slot
                    .channels
                    .video
                    .borrow()
                    .clone()
                    .filter(|f| slot.streaming && f.source_id == binding.source.source_id);
                Some(CameraBindingState {
                    role: *role,
                    source_id: binding.source.source_id.clone(),
                    color_profile_key: binding.color_profile_key.clone(),
                    enabled: binding.enabled,
                    streaming: slot.streaming,
                    has_signal: frame.is_some(),
                    last_sequence: frame.as_ref().map(|f| f.sequence),
                    last_frame_time_ns: frame.as_ref().map(|f| f.received_time_ns),
                    frame: frame.as_ref().map(|f| ImageFrameInfo {
                        width: f.color.width,
                        height: f.color.height,
                        encoding: f.color.pixel_format.clone(),
                        frame_id: f.color.frame_id.clone(),
                    }),
                    original_error: slot.error.clone(),
                })
            })
            .collect()
    }
}
