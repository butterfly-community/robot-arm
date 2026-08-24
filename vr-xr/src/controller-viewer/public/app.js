import * as THREE from "https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js";
import {
  averagePoseSamples,
  frameFacingBaseStation,
  normalizeQuaternion,
  quaternionAxisAngle,
  relativeQuaternionToModel,
  subtract,
  transformPosition,
} from "./pose-math.js";

const $ = (id) => document.getElementById(id);
const HOLD_MS = 6000;
const FUSION_CALIBRATION_DELAY_MS = 1000;
const ORIGIN_WINDOW_MS = 1200;
const MIN_ORIGIN_SAMPLES = 8;
const MAX_SAMPLES = 400;
const circumference = 2 * Math.PI * 49;

let latest = null;
let lastPoseAt = 0;
let reference = null;
let positionFrame = null;
let menuDownAt = null;
let menuWasLong = false;
let previousMenuPressed = false;
let originHoldSamples = [];
let originFusionCalibrationStatus = "idle";
let originFusionCalibrationAttempt = 0;
let relativeOrientation = [0, 0, 0, 1];
let relativePosition = [0, 0, 0];
let positionTrail = [];
let positionTrailRevision = 0;
let lastTrailAt = 0;
let selectedSource = 0;
let simulationRequested = false;
let simulationActive = false;

const SOURCE_LABELS = ["手柄 1", "手柄 2", "头部"];
const BASE_STATION_FRAME = {
  right: [1, 0, 0],
  up: [0, 1, 0],
  forward: [0, 0, -1],
  distance: 0,
};

function createViewState(sourceId) {
  const head = sourceId === 2;
  return {
    latest: null,
    lastPoseAt: 0,
    reference: head
      ? { position: [0, 0, 0], orientation: [0, 0, 0, 1], timeNs: 0 }
      : null,
    positionFrame: head ? BASE_STATION_FRAME : null,
    menuDownAt: null,
    menuWasLong: false,
    previousMenuPressed: false,
    originHoldSamples: [],
    originFusionCalibrationStatus: "idle",
    originFusionCalibrationAttempt: 0,
    relativeOrientation: [0, 0, 0, 1],
    relativePosition: [0, 0, 0],
    positionTrail: [],
    positionTrailRevision: 0,
    lastTrailAt: 0,
  };
}

const viewStates = Array.from(
  { length: 3 },
  (_, sourceId) => createViewState(sourceId),
);

function saveViewState() {
  Object.assign(viewStates[selectedSource], {
    latest,
    lastPoseAt,
    reference,
    positionFrame,
    menuDownAt,
    menuWasLong,
    previousMenuPressed,
    originHoldSamples,
    originFusionCalibrationStatus,
    originFusionCalibrationAttempt,
    relativeOrientation,
    relativePosition,
    positionTrail,
    positionTrailRevision,
    lastTrailAt,
  });
}

function loadViewState() {
  const state = viewStates[selectedSource];
  latest = state.latest;
  lastPoseAt = state.lastPoseAt;
  reference = state.reference;
  positionFrame = state.positionFrame;
  menuDownAt = state.menuDownAt;
  menuWasLong = state.menuWasLong;
  previousMenuPressed = state.previousMenuPressed;
  originHoldSamples = state.originHoldSamples;
  originFusionCalibrationStatus = state.originFusionCalibrationStatus;
  originFusionCalibrationAttempt = state.originFusionCalibrationAttempt;
  relativeOrientation = state.relativeOrientation;
  relativePosition = state.relativePosition;
  positionTrail = state.positionTrail;
  positionTrailRevision = state.positionTrailRevision + 1;
  lastTrailAt = state.lastTrailAt;
}

function sourceId(frame) {
  const value = Number(frame?.source_id ?? frame?.controller_id ?? 0);
  return Number.isInteger(value) && value >= 0 && value < 3 ? value : 0;
}

function cancelCalibrationHold() {
  if (
    menuDownAt !== null ||
    originFusionCalibrationStatus === "waiting" ||
    originFusionCalibrationStatus === "pending"
  ) {
    originFusionCalibrationAttempt += 1;
  }
  menuDownAt = null;
  menuWasLong = false;
  previousMenuPressed = false;
  originHoldSamples = [];
  originFusionCalibrationStatus = "idle";
}

function selectSource(nextSource) {
  if (nextSource === selectedSource) return;
  cancelCalibrationHold();
  saveViewState();
  selectedSource = nextSource;
  loadViewState();
  document.querySelectorAll(".source-button").forEach((button) => {
    const active = Number(button.dataset.source) === selectedSource;
    button.classList.toggle("active", active);
    button.setAttribute("aria-pressed", String(active));
  });
  $("device-title").textContent = `NOLO CV1 ${SOURCE_LABELS[selectedSource]}`;
  $("orientation-title").textContent = selectedSource === 2
    ? "头部方向"
    : "手柄方向";
  $("model-front").innerHTML = selectedSource === 2
    ? "<i></i>面部 / 前"
    : "<i></i>头部 / 前";
  $("model-top").innerHTML = selectedSource === 2
    ? "<i></i>头顶 / 上"
    : "<i></i>触摸板 / 上";
  $("model-back").innerHTML = selectedSource === 2
    ? "<i></i>后脑 / 后"
    : "<i></i>尾部 / 后";
  document.querySelector(".hold-meter").hidden = selectedSource === 2;
  if (selectedSource !== 2) {
    setCalibrationMessage(
      positionFrame ? "标定已保留" : "记录原点和零姿态",
      positionFrame
        ? "当前手柄的独立标定仍然有效；连续按住菜单键 6 秒可重新标定。"
        : "人和手柄正对基站，触摸板朝上、手柄头部指向基站，静止长按菜单键 6 秒。",
    );
    $("calibration-state").textContent = positionFrame ? "已完成" : "未开始";
  }
  if (latest) {
    updateFrame(latest, false);
  } else {
    $("connection").textContent = `等待${SOURCE_LABELS[selectedSource]}数据`;
    $("connection").classList.add("waiting");
  }
}

document.querySelectorAll(".source-button").forEach((button) => {
  button.addEventListener(
    "click",
    () => selectSource(Number(button.dataset.source)),
  );
});

function fmt(values, digits = 5) {
  return `[${values.map((value) => Number(value).toFixed(digits)).join(", ")}]`;
}

function directionText(value, positive, negative) {
  const centimeters = Math.abs(value) * 100;
  if (centimeters < 0.5) return `居中 ${centimeters.toFixed(1)} 厘米`;
  return `${value >= 0 ? positive : negative} ${centimeters.toFixed(1)} 厘米`;
}

function validPose(frame) {
  return frame && frame.pose_usable === true &&
    frame.communication_fresh === true &&
    Array.isArray(frame.position) && frame.position.length === 3 &&
    Array.isArray(frame.orientation) && frame.orientation.length === 4 &&
    frame.position.every(Number.isFinite) &&
    frame.orientation.every(Number.isFinite);
}

function poseSample(frame) {
  return {
    position: [...frame.position],
    orientation: normalizeQuaternion([...frame.orientation]),
    capturedAt: performance.now(),
  };
}

function appendBounded(samples, sample) {
  samples.push(sample);
  if (samples.length > MAX_SAMPLES) samples.shift();
}

function setCalibrationMessage(title, help) {
  $("calibration-title").textContent = title;
  $("calibration-help").textContent = help;
}

async function requestOriginFusionCalibration(sourceId, attempt) {
  let status = "accepted";
  try {
    const response = await fetch(`/api/pose-calibration/${sourceId}`, {
      method: "POST",
    });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
  } catch (error) {
    status = "failed";
    if (
      sourceId === selectedSource && attempt === originFusionCalibrationAttempt
    ) {
      setCalibrationMessage(
        "Fusion 标定启动失败",
        `后端请求失败：${error.message}。请松开菜单键后重试。`,
      );
    }
  }

  const state = viewStates[sourceId];
  if (state.originFusionCalibrationAttempt === attempt) {
    state.originFusionCalibrationStatus = status;
  }
  if (
    sourceId === selectedSource && attempt === originFusionCalibrationAttempt
  ) {
    originFusionCalibrationStatus = status;
  }
}

function localizedDeviceName(name) {
  return String(name)
    .replace("Controller", "控制器")
    .replace("(Virtual USB)", "（虚拟 USB 报告）")
    .replace("(USB)", "（USB 直连）");
}

function updateRelative(frame) {
  if (!reference || !validPose(frame)) return;
  relativePosition = transformPosition(
    frame.position,
    reference.position,
    positionFrame,
  );
  relativeOrientation = relativeQuaternionToModel(
    reference.orientation,
    frame.orientation,
  );

  const rawDelta = subtract(frame.position, reference.position);
  const distanceCm = Math.hypot(...relativePosition) * 100;
  $("raw-delta").textContent = fmt(rawDelta);
  $("relative-position").textContent = fmt(relativePosition);
  $("relative-distance").textContent = `${distanceCm.toFixed(1)} 厘米`;

  if (positionFrame) {
    $("position-lr").textContent = directionText(
      relativePosition[0],
      "右",
      "左",
    );
    $("position-ud").textContent = directionText(
      relativePosition[1],
      "上",
      "下",
    );
    $("position-fb").textContent = directionText(
      -relativePosition[2],
      "前",
      "后",
    );
    $("position-distance").textContent = `${distanceCm.toFixed(1)} 厘米`;
    const now = performance.now();
    const previous = positionTrail.at(-1);
    const moved = previous
      ? Math.hypot(...subtract(relativePosition, previous))
      : Infinity;
    if (moved >= 0.005 || now - lastTrailAt >= 250) {
      positionTrail.push([...relativePosition]);
      if (positionTrail.length > 240) positionTrail.shift();
      positionTrailRevision++;
      lastTrailAt = now;
    }
  } else {
    $("position-lr").textContent = "待标定";
    $("position-ud").textContent = "待标定";
    $("position-fb").textContent = "待标定";
    $("position-distance").textContent = `${distanceCm.toFixed(1)} 厘米`;
  }

  $("relative-orientation").textContent = fmt(relativeOrientation);
  const { axis, degrees } = quaternionAxisAngle(relativeOrientation);
  $("relative-axis").textContent = fmt(axis);
  $("relative-angle").textContent = `${degrees.toFixed(2)}°`;
  $("orientation-summary").textContent = `相对旋转 ${degrees.toFixed(1)}°`;
}

function completeOrigin() {
  if (!validPose(latest)) return;
  const fusionReady = originFusionCalibrationStatus === "accepted" &&
    latest.gyro_calibration_complete === true &&
    latest.gyro_calibration_active !== true &&
    latest.fusion_initialising !== true;
  if (!fusionReady) {
    const reason = originFusionCalibrationStatus === "waiting"
      ? "尚未进入第二秒的 Fusion 标定阶段"
      : originFusionCalibrationStatus === "failed"
      ? "Fusion 标定请求失败"
      : originFusionCalibrationStatus === "pending"
      ? "Fusion 标定请求仍在处理"
      : latest.gyro_calibration_active
      ? "设备在 6 秒内没有连续静止满 3 秒"
      : latest.fusion_initialising
      ? "Fusion 初始化尚未完成"
      : "Fusion 标定状态无效";
    setCalibrationMessage(
      "本次标定未完成",
      `${reason}；请松开菜单键，保持手柄静止后重新长按 6 秒。`,
    );
    $("calibration-state").textContent = "Fusion 标定未完成";
    menuWasLong = true;
    return;
  }
  const cutoff = performance.now() - ORIGIN_WINDOW_MS;
  let samples = originHoldSamples.filter((sample) =>
    sample.capturedAt >= cutoff
  );
  if (samples.length < MIN_ORIGIN_SAMPLES) samples = [...originHoldSamples];
  if (samples.length < MIN_ORIGIN_SAMPLES) {
    setCalibrationMessage(
      "原点采样不足",
      `只收到 ${samples.length} 个有效样本，请松开后重新按住菜单键 6 秒。`,
    );
    menuWasLong = true;
    return;
  }

  const averaged = averagePoseSamples(samples);
  let baseFrame;
  try {
    baseFrame = frameFacingBaseStation(averaged.position);
  } catch {
    setCalibrationMessage(
      "离基站太近，无法标定",
      "请退到离基站至少 50 厘米处，人与手柄都正对基站后重新长按菜单键 6 秒。",
    );
    $("calibration-state").textContent = "基站方向不可用";
    menuWasLong = true;
    return;
  }
  reference = {
    position: averaged.position,
    orientation: averaged.orientation,
    timeNs: latest.time_ns,
  };
  positionFrame = baseFrame;
  positionTrail = [[0, 0, 0]];
  positionTrailRevision++;
  lastTrailAt = 0;
  menuWasLong = true;
  $("origin-samples").textContent = String(averaged.sampleCount);
  $("origin-jitter").textContent = `${
    (averaged.rmsMeters * 1000).toFixed(1)
  } 毫米均方根`;
  $("forward-source").textContent = "当前手柄位置指向基站原点";
  $("forward-fit").textContent = `${baseFrame.distance.toFixed(3)} 米`;
  $("frame-basis").textContent = `右 ${fmt(baseFrame.right, 3)} / 上 ${
    fmt(baseFrame.up, 3)
  } / 前 ${fmt(baseFrame.forward, 3)}`;
  setCalibrationMessage(
    "标定完成",
    "已将朝向基站定义为前方，并记录原点和零姿态；已有陀螺仪零偏继续复用。现在可以松开菜单键。",
  );
  $("calibration-state").textContent = "已完成（正对基站）";
  updateRelative(latest);
}

function updateFrame(frame, processMenu = true) {
  latest = frame;
  lastPoseAt = performance.now();
  const isHead = selectedSource === 2;
  const valid = validPose(frame);
  $("connection").textContent = valid ? "新采样已连接" : "采样不可用";
  $("connection").classList.toggle("waiting", !valid);
  $("device").textContent = localizedDeviceName(frame.device);
  $("flags").textContent = `0x${
    Number(frame.flags).toString(16).padStart(2, "0")
  }（仅兼容保留）`;
  $("raw-position").textContent = fmt(frame.position);
  $("filtered-position").textContent = Array.isArray(frame.filtered_position)
    ? fmt(frame.filtered_position)
    : isHead
    ? "头部不适用"
    : "等待新的有效位置样本";
  $("position-mode").textContent = frame.position_mode === "grip"
    ? "估算握持点（实验）"
    : "原始光学标记";
  $("marker-position").textContent = Array.isArray(frame.marker_position)
    ? fmt(frame.marker_position)
    : "数据未提供";
  $("grip-position").textContent = Array.isArray(frame.grip_position)
    ? fmt(frame.grip_position)
    : "头部不适用";
  $("optical-valid").textContent = frame.optical_tracking_valid == null
    ? "协议未识别，不能据此控制机械臂"
    : frame.optical_tracking_valid
    ? "有效"
    : "无效";
  $("raw-orientation").textContent = fmt(frame.orientation);
  $("menu-state").textContent = isHead
    ? "头部无菜单键"
    : !frame.menu_active
    ? "输入未激活"
    : frame.menu_pressed
    ? "按下"
    : "松开";
  const unchangedMs = Number(frame.unchanged_ms ?? 0);
  $("pose-unchanged").textContent = Number.isFinite(unchangedMs)
    ? `${(unchangedMs / 1000).toFixed(1)} 秒`
    : "未知";
  $("source-online").textContent = frame.source_online
    ? "序号持续更新"
    : "序号停止：关机、休眠或失联";
  $("hmd-relay-online").textContent = frame.hmd_relay_online
    ? "序号持续更新"
    : "序号停止";
  const hmdUnchangedMs = Number(frame.hmd_unchanged_ms ?? 0);
  $("hmd-unchanged").textContent = Number.isFinite(hmdUnchangedMs)
    ? `${(hmdUnchangedMs / 1000).toFixed(1)} 秒`
    : "未知";
  $("sample-sequences").textContent = isHead
    ? `— / ${Number(frame.hmd_sequence ?? 0)}（约 ${
      Number(frame.sample_rate_hz ?? 240)
    } Hz）`
    : `${Number(frame.controller_sequence ?? frame.sample_sequence ?? 0)} / ${
      Number(frame.hmd_sequence ?? 0)
    }（约 ${Number(frame.sample_rate_hz ?? 120)} Hz）`;
  const measuredRate = Number(frame.measured_rate_hz);
  const jitter = Number(frame.sample_jitter_ms);
  $("sample-quality").textContent = Number.isFinite(measuredRate)
    ? `${measuredRate.toFixed(2)} Hz / ${
      Number.isFinite(jitter) ? jitter.toFixed(3) : "—"
    } ms`
    : "等待第二个新样本";
  $("sample-counts").textContent = `${Number(frame.samples_received ?? 0)} / ${
    Number(frame.samples_missed ?? 0)
  } / ${Number(frame.duplicate_reports ?? 0)}`;
  const accelerationError = Number(frame.acceleration_error_degrees);
  $("acceleration-state").textContent = frame.accelerometer_ignored
    ? `暂时拒绝（误差 ${accelerationError.toFixed(2)}°）`
    : frame.acceleration_recovery
    ? `恢复中（误差 ${accelerationError.toFixed(2)}°）`
    : `已采用（误差 ${accelerationError.toFixed(2)}°）`;
  $("gyro-bias").textContent = Array.isArray(frame.gyro_bias_dps)
    ? fmt(frame.gyro_bias_dps, 4)
    : "—";
  const calibrationProgress = Math.max(
    0,
    Math.min(1, Number(frame.gyro_calibration_progress ?? 0)),
  );
  $("gyro-calibration-state").textContent = frame.gyro_calibration_active
    ? `保持静止 ${(calibrationProgress * 100).toFixed(0)}%`
    : frame.gyro_calibration_complete
    ? "已完成"
    : "未开始";
  $("pose-unchanged-label").textContent = isHead
    ? "头部序号未变化"
    : "控制器序号未变化";
  $("source-online-label").textContent = `${SOURCE_LABELS[selectedSource]}通信`;
  $("menu-state-label").textContent = isHead ? "输入" : "菜单键";
  if (valid && unchangedMs >= 2000) {
    $("connection").textContent = "位姿未变化：静止或数据冻结";
    $("connection").classList.add("waiting");
  }

  if (isHead) {
    setCalibrationMessage(
      "基站跟踪坐标",
      "头部没有菜单键；当前位置直接按 NOLO 基站坐标显示，姿态零点为后端启动时的 Fusion 初始状态。",
    );
    $("calibration-state").textContent = "基站坐标（未做人类前方重置）";
    $("forward-source").textContent = "NOLO 基站跟踪轴";
    $("frame-basis").textContent = "右 +X / 上 +Y / 前 −Z";
    $("origin-samples").textContent = "不适用";
    $("origin-jitter").textContent = "不适用";
    $("forward-fit").textContent = "不适用";
  }

  const pressed = !isHead && valid && frame.menu_pressed;
  if (!valid) {
    cancelCalibrationHold();
  } else if (processMenu && !isHead) {
    const sample = poseSample(frame);
    if (pressed) appendBounded(originHoldSamples, sample);
    if (pressed && !previousMenuPressed) {
      menuDownAt = performance.now();
      menuWasLong = false;
      originHoldSamples = [sample];
      originFusionCalibrationAttempt += 1;
      originFusionCalibrationStatus = "waiting";
      viewStates[selectedSource].originFusionCalibrationAttempt =
        originFusionCalibrationAttempt;
      viewStates[selectedSource].originFusionCalibrationStatus = "waiting";
      setCalibrationMessage(
        positionFrame ? "继续按住 6 秒可重新标定" : "保持标定姿势并继续按住",
        latest.gyro_calibration_complete
          ? "陀螺仪零偏已从文件加载或此前完成，本次只重新记录原点和零姿态。"
          : "尚无零偏文件；第一秒用于摆稳，随后只在本次完成零偏并写入文件，最后约 1.2 秒记录原点和零姿态。",
      );
    } else if (!pressed && previousMenuPressed) {
      if (!menuWasLong) {
        setCalibrationMessage(
          positionFrame ? "标定保持不变" : "按住时间不足",
          `短按不会改变标定；${
            positionFrame ? "重新标定" : "开始标定"
          }请连续按住菜单键 6 秒。`,
        );
      }
      cancelCalibrationHold();
    }
    previousMenuPressed = pressed;
  }
  if (valid) updateRelative(frame);
}

function receiveFrame(frame) {
  const id = sourceId(frame);
  if (frame.simulated === true) {
    const active = frame.communication_fresh === true;
    setSimulationState(active, active);
  } else if (frame.communication_fresh === true && !simulationRequested) {
    setSimulationState(false, false);
  }
  viewStates[id].latest = frame;
  viewStates[id].lastPoseAt = performance.now();
  if (id === selectedSource) updateFrame(frame);
}

function setSimulationState(requested, active) {
  simulationRequested = requested;
  simulationActive = active;
  const button = $("simulation-toggle");
  button.textContent = requested || active ? "停止模拟" : "启动模拟";
  button.classList.toggle("active", active);
  button.setAttribute("aria-pressed", String(active));
}

function resetForAutomaticSimulationCalibration() {
  if (selectedSource !== 0) selectSource(0);
  viewStates[0] = createViewState(0);
  loadViewState();
  setCalibrationMessage(
    "模拟数据自动标定中",
    "虚拟手柄会保持静止并自动长按 Menu 6 秒；无需操作真实手柄。标定和接管门槛完成后会先预抬升 20 厘米，再开始循环。",
  );
  $("calibration-state").textContent = "等待虚拟 Menu 长按";
  $("position-lr").textContent = "自动标定中";
  $("position-ud").textContent = "自动标定中";
  $("position-fb").textContent = "自动标定中";
  $("position-distance").textContent = "自动标定中";
}

async function refreshSimulationState() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const payload = await response.json();
    setSimulationState(
      payload.simulationRequested === true,
      payload.simulationActive === true,
    );
  } catch {
    // SSE connection state already reports server availability.
  }
}

$("simulation-toggle").addEventListener("click", async () => {
  const button = $("simulation-toggle");
  const start = !(simulationRequested || simulationActive);
  button.disabled = true;
  button.textContent = start ? "正在启动…" : "正在停止…";
  try {
    const response = await fetch(
      start ? "/api/simulation/start" : "/api/simulation/stop",
      { method: "POST" },
    );
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(
        payload.error ?? payload.status ?? `HTTP ${response.status}`,
      );
    }
    setSimulationState(start, false);
    if (start) {
      resetForAutomaticSimulationCalibration();
    } else {
      $("moveit-restart-dialog").showModal();
    }
    await refreshSimulationState();
  } catch (error) {
    $("connection").textContent = `模拟切换失败：${error.message}`;
    $("connection").classList.add("waiting");
    await refreshSimulationState();
  } finally {
    button.disabled = false;
  }
});

void refreshSimulationState();

const source = new EventSource("/events");
source.addEventListener("status", (event) => {
  const { status } = JSON.parse(event.data);
  if (
    (simulationRequested || simulationActive) &&
    String(status).startsWith("NOLO 虚拟 USB：")
  ) {
    // Phase changes are normal virtual samples, not connection failures.  The
    // next pose arrives within one report period.  Do not touch the badge at
    // all: even a brief phase label changes its width and reflows the embedded
    // dashboard at every three-second action edge.
    return;
  }
  if (!latest || status !== "原始 USB 数据已连接") {
    $("connection").textContent = status;
    $("connection").classList.add("waiting");
  }
});
source.addEventListener(
  "pose",
  (event) => receiveFrame(JSON.parse(event.data)),
);
source.onerror = () => {
  $("connection").textContent = "网页数据连接中断";
  $("connection").classList.add("waiting");
};

$("gyro-calibrate").addEventListener("click", async () => {
  const button = $("gyro-calibrate");
  button.disabled = true;
  $("gyro-calibration-state").textContent = "正在提交…";
  try {
    const response = await fetch(`/api/gyro-calibration/${selectedSource}`, {
      method: "POST",
    });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    $("gyro-calibration-state").textContent = "已提交，请保持静止";
  } catch (error) {
    $("gyro-calibration-state").textContent = `提交失败：${error.message}`;
  } finally {
    button.disabled = false;
  }
});

setInterval(() => {
  if (lastPoseAt && performance.now() - lastPoseAt > 500) {
    $("connection").textContent = "姿态数据超时";
    $("connection").classList.add("waiting");
  }
}, 250);

function animateHold(now) {
  if (selectedSource === 2) {
    requestAnimationFrame(animateHold);
    return;
  }
  let held = menuDownAt === null ? 0 : Math.min(HOLD_MS, now - menuDownAt);
  if (
    held >= FUSION_CALIBRATION_DELAY_MS &&
    originFusionCalibrationStatus === "waiting"
  ) {
    if (
      latest?.gyro_calibration_complete === true &&
      latest?.gyro_calibration_active !== true
    ) {
      originFusionCalibrationStatus = "accepted";
      viewStates[selectedSource].originFusionCalibrationStatus = "accepted";
    } else {
      originFusionCalibrationStatus = "pending";
      viewStates[selectedSource].originFusionCalibrationStatus = "pending";
      void requestOriginFusionCalibration(
        selectedSource,
        originFusionCalibrationAttempt,
      );
    }
  }
  if (held >= HOLD_MS && !menuWasLong) completeOrigin();
  if (menuWasLong && latest?.menu_pressed) held = HOLD_MS;
  const ratio = held / HOLD_MS;
  $("hold-seconds").textContent = (held / 1000).toFixed(1);
  $("meter-progress").style.strokeDashoffset = String(
    circumference * (1 - ratio),
  );
  $("meter-progress").parentElement.parentElement.setAttribute(
    "aria-valuenow",
    (held / 1000).toFixed(1),
  );
  requestAnimationFrame(animateHold);
}
requestAnimationFrame(animateHold);

function createControllerModel() {
  const group = new THREE.Group();
  const shell = new THREE.MeshStandardMaterial({
    color: 0x263842,
    roughness: 0.58,
    metalness: 0.12,
  });
  const dark = new THREE.MeshStandardMaterial({
    color: 0x111a20,
    roughness: 0.72,
  });
  const cyan = new THREE.MeshStandardMaterial({
    color: 0x35cfd4,
    emissive: 0x0b4e50,
    emissiveIntensity: 0.8,
  });
  const amber = new THREE.MeshStandardMaterial({
    color: 0xffb24e,
    emissive: 0x5a3000,
    emissiveIntensity: 0.65,
  });
  const violet = new THREE.MeshStandardMaterial({
    color: 0x846ee0,
    emissive: 0x23174f,
    emissiveIntensity: 0.45,
  });

  const grip = new THREE.Mesh(
    new THREE.CylinderGeometry(0.018, 0.026, 0.125, 32, 4),
    shell,
  );
  grip.rotation.x = Math.PI / 2;
  grip.position.set(0, -0.004, 0.026);
  group.add(grip);
  const head = new THREE.Mesh(new THREE.SphereGeometry(1, 40, 24), shell);
  head.scale.set(0.038, 0.029, 0.046);
  head.position.set(0, 0.002, -0.063);
  group.add(head);
  const face = new THREE.Mesh(
    new THREE.CylinderGeometry(0.024, 0.024, 0.007, 40),
    dark,
  );
  face.position.set(0, 0.029, -0.044);
  group.add(face);
  const pad = new THREE.Mesh(
    new THREE.CylinderGeometry(0.0205, 0.0205, 0.008, 40),
    violet,
  );
  pad.position.set(0, 0.033, -0.044);
  group.add(pad);
  const menu = new THREE.Mesh(
    new THREE.CylinderGeometry(0.006, 0.006, 0.008, 24),
    dark,
  );
  menu.position.set(0, 0.027, 0.002);
  group.add(menu);
  const headMarker = new THREE.Mesh(
    new THREE.ConeGeometry(0.010, 0.031, 24),
    cyan,
  );
  headMarker.rotation.x = -Math.PI / 2;
  headMarker.position.set(0, 0, -0.111);
  group.add(headMarker);
  const tailMarker = new THREE.Mesh(
    new THREE.TorusGeometry(0.015, 0.0035, 12, 32),
    amber,
  );
  tailMarker.position.set(0, -0.006, 0.094);
  group.add(tailMarker);
  return group;
}

function createHeadModel() {
  const group = new THREE.Group();
  const shell = new THREE.MeshStandardMaterial({
    color: 0x263842,
    roughness: 0.52,
    metalness: 0.14,
  });
  const glass = new THREE.MeshStandardMaterial({
    color: 0x172d3c,
    emissive: 0x0b3950,
    emissiveIntensity: 0.45,
    roughness: 0.2,
    metalness: 0.3,
  });
  const cyan = new THREE.MeshStandardMaterial({
    color: 0x35cfd4,
    emissive: 0x0b4e50,
    emissiveIntensity: 0.8,
  });
  const visor = new THREE.Mesh(
    new THREE.BoxGeometry(0.15, 0.075, 0.075),
    shell,
  );
  visor.position.z = -0.02;
  group.add(visor);
  const face = new THREE.Mesh(
    new THREE.BoxGeometry(0.135, 0.055, 0.009),
    glass,
  );
  face.position.z = -0.061;
  group.add(face);
  const topMarker = new THREE.Mesh(
    new THREE.ConeGeometry(0.012, 0.035, 24),
    cyan,
  );
  topMarker.position.set(0, 0.06, -0.02);
  group.add(topMarker);
  const band = new THREE.Mesh(
    new THREE.TorusGeometry(0.083, 0.008, 12, 48, Math.PI),
    shell,
  );
  band.rotation.set(Math.PI / 2, 0, Math.PI / 2);
  band.position.z = 0.02;
  group.add(band);
  return group;
}

function addArrow(scene, direction, color) {
  scene.add(
    new THREE.ArrowHelper(
      direction,
      new THREE.Vector3(),
      0.19,
      color,
      0.025,
      0.012,
    ),
  );
}

function roundedRectangle(context, x, y, width, height, radius) {
  context.beginPath();
  context.moveTo(x + radius, y);
  context.lineTo(x + width - radius, y);
  context.quadraticCurveTo(x + width, y, x + width, y + radius);
  context.lineTo(x + width, y + height - radius);
  context.quadraticCurveTo(
    x + width,
    y + height,
    x + width - radius,
    y + height,
  );
  context.lineTo(x + radius, y + height);
  context.quadraticCurveTo(x, y + height, x, y + height - radius);
  context.lineTo(x, y + radius);
  context.quadraticCurveTo(x, y, x + radius, y);
  context.closePath();
}

function createTextSprite(text, color = "#e8edf2") {
  const canvas = document.createElement("canvas");
  const measure = canvas.getContext("2d");
  measure.font = '700 28px "Noto Sans SC", system-ui, sans-serif';
  canvas.width = Math.ceil(measure.measureText(text).width + 52);
  canvas.height = 64;
  const context = canvas.getContext("2d");
  context.font = '700 28px "Noto Sans SC", system-ui, sans-serif';
  roundedRectangle(context, 2, 2, canvas.width - 4, canvas.height - 4, 15);
  context.fillStyle = "rgba(5, 12, 18, 0.88)";
  context.fill();
  context.strokeStyle = "rgba(255, 255, 255, 0.16)";
  context.lineWidth = 2;
  context.stroke();
  context.fillStyle = color;
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText(text, canvas.width / 2, canvas.height / 2 + 1);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const material = new THREE.SpriteMaterial({
    map: texture,
    transparent: true,
    depthTest: false,
    depthWrite: false,
    toneMapped: false,
  });
  const sprite = new THREE.Sprite(material);
  sprite.renderOrder = 20;
  sprite.userData.labelAspect = canvas.width / canvas.height;
  return sprite;
}

function scaleTextSprite(sprite, cameraDistance, heightFactor = 0.047) {
  const height = cameraDistance * heightFactor;
  sprite.scale.set(height * sprite.userData.labelAspect, height, 1);
}

const poseCanvas = $("pose-canvas");
try {
  const renderer = new THREE.WebGLRenderer({
    canvas: poseCanvas,
    antialias: true,
    alpha: true,
  });
  renderer.setPixelRatio(Math.min(globalThis.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(36, 1, 0.01, 10);
  camera.position.set(0.46, 0.34, 0.64);
  camera.lookAt(0, 0, 0);
  scene.add(new THREE.HemisphereLight(0xc8efff, 0x182029, 2.1));
  const key = new THREE.DirectionalLight(0xffffff, 2.7);
  key.position.set(0.35, 0.55, 0.45);
  scene.add(key);
  addArrow(scene, new THREE.Vector3(1, 0, 0), 0xff7378);
  addArrow(scene, new THREE.Vector3(-1, 0, 0), 0x7c3034);
  addArrow(scene, new THREE.Vector3(0, 1, 0), 0x77e49b);
  addArrow(scene, new THREE.Vector3(0, -1, 0), 0x315f41);
  addArrow(scene, new THREE.Vector3(0, 0, -1), 0x65b9ff);
  addArrow(scene, new THREE.Vector3(0, 0, 1), 0x294d69);
  const controllerModel = createControllerModel();
  const headModel = createHeadModel();
  scene.add(controllerModel);
  scene.add(headModel);
  let renderedWidth = 0;
  let renderedHeight = 0;

  const renderThree = () => {
    const width = Math.max(1, poseCanvas.clientWidth);
    const height = Math.max(1, poseCanvas.clientHeight);
    if (width !== renderedWidth || height !== renderedHeight) {
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderedWidth = width;
      renderedHeight = height;
    }
    controllerModel.visible = selectedSource !== 2;
    headModel.visible = selectedSource === 2;
    controllerModel.quaternion.set(...relativeOrientation);
    headModel.quaternion.set(...relativeOrientation);
    renderer.render(scene, camera);
    requestAnimationFrame(renderThree);
  };
  requestAnimationFrame(renderThree);
} catch {
  poseCanvas.replaceWith(Object.assign(document.createElement("p"), {
    className: "webgl-error",
    textContent: "三维图形初始化失败，请确认浏览器已启用硬件加速。",
  }));
}

const spatialCanvas = $("spatial-canvas");
try {
  const renderer = new THREE.WebGLRenderer({
    canvas: spatialCanvas,
    antialias: true,
    alpha: true,
  });
  renderer.setPixelRatio(Math.min(globalThis.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.15;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const scene = new THREE.Scene();
  scene.fog = new THREE.Fog(0x081018, 3.4, 8);
  const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 20);
  camera.position.set(0.8, 0.58, 0.9);
  const cameraTarget = new THREE.Vector3(0, 0.08, 0);

  scene.add(new THREE.HemisphereLight(0xbdeeff, 0x10171d, 1.8));
  const spatialKey = new THREE.DirectionalLight(0xffffff, 3.2);
  spatialKey.position.set(1.4, 2.2, 1.3);
  spatialKey.castShadow = true;
  spatialKey.shadow.mapSize.set(1024, 1024);
  spatialKey.shadow.camera.near = 0.1;
  spatialKey.shadow.camera.far = 8;
  spatialKey.shadow.camera.left = -2;
  spatialKey.shadow.camera.right = 2;
  spatialKey.shadow.camera.top = 2;
  spatialKey.shadow.camera.bottom = -2;
  scene.add(spatialKey);
  const rim = new THREE.PointLight(0x44dce2, 7, 3, 2);
  rim.position.set(-0.7, 0.7, -0.8);
  scene.add(rim);

  const zeroPlane = new THREE.Mesh(
    new THREE.PlaneGeometry(4, 4),
    new THREE.MeshStandardMaterial({
      color: 0x0a151d,
      transparent: true,
      opacity: 0.58,
      roughness: 0.94,
      metalness: 0.02,
    }),
  );
  zeroPlane.rotation.x = -Math.PI / 2;
  zeroPlane.position.y = -0.006;
  zeroPlane.receiveShadow = true;
  scene.add(zeroPlane);

  const grid = new THREE.GridHelper(4, 40, 0x315d68, 0x172a32);
  grid.position.y = -0.003;
  grid.material.transparent = true;
  grid.material.opacity = 0.72;
  scene.add(grid);

  const originRing = new THREE.Mesh(
    new THREE.RingGeometry(0.045, 0.065, 48),
    new THREE.MeshBasicMaterial({
      color: 0xffbd66,
      transparent: true,
      opacity: 0.92,
      side: THREE.DoubleSide,
    }),
  );
  originRing.rotation.x = -Math.PI / 2;
  originRing.position.y = 0.003;
  scene.add(originRing);
  const originCore = new THREE.Mesh(
    new THREE.CylinderGeometry(0.012, 0.018, 0.028, 24),
    new THREE.MeshStandardMaterial({
      color: 0xffbd66,
      emissive: 0x6a3500,
      emissiveIntensity: 0.8,
    }),
  );
  originCore.position.y = 0.014;
  scene.add(originCore);

  const spatialAxes = new THREE.Group();
  spatialAxes.add(
    new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(),
      0.30,
      0xff7378,
      0.045,
      0.024,
    ),
    new THREE.ArrowHelper(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(),
      0.30,
      0x77e49b,
      0.045,
      0.024,
    ),
    new THREE.ArrowHelper(
      new THREE.Vector3(0, 0, -1),
      new THREE.Vector3(),
      0.30,
      0x65b9ff,
      0.045,
      0.024,
    ),
  );
  scene.add(spatialAxes);

  const spatialLabels = [
    {
      sprite: createTextSprite("右方 +X", "#ff9ca0"),
      position: [0.39, 0.035, 0],
    },
    { sprite: createTextSprite("上方 +Y", "#9df4b7"), position: [0, 0.39, 0] },
    {
      sprite: createTextSprite("前方 −Z", "#9bd4ff"),
      position: [0, 0.035, -0.39],
    },
    {
      sprite: createTextSprite("标定原点", "#ffd18f"),
      position: [-0.02, 0.075, 0.11],
    },
  ];
  for (const label of spatialLabels) {
    label.sprite.position.set(...label.position);
    scene.add(label.sprite);
  }
  const controllerLabel = createTextSprite("实时设备", "#85f4f5");
  scene.add(controllerLabel);

  const spatialController = createControllerModel();
  spatialController.traverse((object) => {
    if (object.isMesh) object.castShadow = true;
  });
  spatialController.visible = false;
  scene.add(spatialController);
  const spatialHead = createHeadModel();
  spatialHead.traverse((object) => {
    if (object.isMesh) object.castShadow = true;
  });
  spatialHead.visible = false;
  scene.add(spatialHead);

  const trailGeometry = new THREE.BufferGeometry();
  const trailLine = new THREE.Line(
    trailGeometry,
    new THREE.LineBasicMaterial({
      color: 0x4bd8dc,
      transparent: true,
      opacity: 0.86,
    }),
  );
  scene.add(trailLine);
  const trailPoints = new THREE.Points(
    trailGeometry,
    new THREE.PointsMaterial({
      color: 0x9ffcff,
      size: 0.014,
      sizeAttenuation: true,
      transparent: true,
      opacity: 0.72,
    }),
  );
  scene.add(trailPoints);

  const projectionGeometry = new THREE.BufferGeometry();
  const projectionLine = new THREE.Line(
    projectionGeometry,
    new THREE.LineDashedMaterial({
      color: 0xffbd66,
      transparent: true,
      opacity: 0.72,
      dashSize: 0.025,
      gapSize: 0.018,
    }),
  );
  scene.add(projectionLine);
  const floorMarker = new THREE.Mesh(
    new THREE.RingGeometry(0.025, 0.038, 36),
    new THREE.MeshBasicMaterial({
      color: 0x4bd8dc,
      transparent: true,
      opacity: 0.9,
      side: THREE.DoubleSide,
    }),
  );
  floorMarker.rotation.x = -Math.PI / 2;
  floorMarker.visible = false;
  scene.add(floorMarker);

  let renderedWidth = 0;
  let renderedHeight = 0;
  let renderedTrailRevision = -1;
  const desiredTarget = new THREE.Vector3();
  const desiredCamera = new THREE.Vector3();
  const viewDirection = new THREE.Vector3(1.08, 0.74, 1.14).normalize();

  const updateSpatialTrail = () => {
    const points = positionTrail.map((point) => new THREE.Vector3(...point));
    if (!points.length) points.push(new THREE.Vector3());
    trailGeometry.setFromPoints(points);
    trailGeometry.computeBoundingSphere();
    renderedTrailRevision = positionTrailRevision;
  };

  const spatialRender = () => {
    const width = Math.max(1, spatialCanvas.clientWidth);
    const height = Math.max(1, spatialCanvas.clientHeight);
    if (width !== renderedWidth || height !== renderedHeight) {
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderedWidth = width;
      renderedHeight = height;
    }
    if (renderedTrailRevision !== positionTrailRevision) updateSpatialTrail();

    const ready = Boolean(positionFrame && reference);
    spatialController.visible = ready && selectedSource !== 2;
    spatialHead.visible = ready && selectedSource === 2;
    controllerLabel.visible = ready;
    floorMarker.visible = ready;
    trailLine.visible = ready && positionTrail.length > 1;
    trailPoints.visible = trailLine.visible;
    projectionLine.visible = ready;

    if (ready) {
      spatialController.position.set(...relativePosition);
      spatialController.quaternion.set(...relativeOrientation);
      spatialHead.position.set(...relativePosition);
      spatialHead.quaternion.set(...relativeOrientation);
      controllerLabel.position.set(
        relativePosition[0],
        relativePosition[1] + 0.14,
        relativePosition[2],
      );
      floorMarker.position.set(relativePosition[0], 0.004, relativePosition[2]);
      projectionGeometry.setFromPoints([
        new THREE.Vector3(relativePosition[0], 0, relativePosition[2]),
        new THREE.Vector3(...relativePosition),
      ]);
      projectionLine.computeLineDistances();
      $("spatial-summary").textContent = `${
        directionText(relativePosition[0], "右", "左")
      } / ${directionText(relativePosition[1], "上", "下")} / ${
        directionText(-relativePosition[2], "前", "后")
      }`;
    } else {
      $("spatial-summary").textContent = "等待完成位置标定";
    }

    const visiblePoints = ready
      ? [[0, 0, 0], ...positionTrail, relativePosition]
      : [[-0.45, 0, -0.45], [0.45, 0.42, 0.45]];
    const minimum = new THREE.Vector3(Infinity, Infinity, Infinity);
    const maximum = new THREE.Vector3(-Infinity, -Infinity, -Infinity);
    for (const point of visiblePoints) {
      minimum.min(new THREE.Vector3(...point));
      maximum.max(new THREE.Vector3(...point));
    }
    minimum.min(new THREE.Vector3(-0.45, -0.05, -0.45));
    maximum.max(new THREE.Vector3(0.45, 0.42, 0.45));
    desiredTarget.addVectors(minimum, maximum).multiplyScalar(0.5);
    const span = Math.max(
      maximum.x - minimum.x,
      maximum.y - minimum.y,
      maximum.z - minimum.z,
    );
    const cameraDistance = Math.max(1.25, span * 1.9);
    desiredCamera.copy(viewDirection).multiplyScalar(cameraDistance).add(
      desiredTarget,
    );
    camera.position.lerp(desiredCamera, 0.08);
    cameraTarget.lerp(desiredTarget, 0.08);
    camera.lookAt(cameraTarget);
    for (const label of spatialLabels) {
      scaleTextSprite(label.sprite, cameraDistance);
    }
    scaleTextSprite(controllerLabel, cameraDistance, 0.042);

    renderer.render(scene, camera);
    requestAnimationFrame(spatialRender);
  };
  requestAnimationFrame(spatialRender);
} catch {
  spatialCanvas.replaceWith(Object.assign(document.createElement("p"), {
    className: "webgl-error",
    textContent: "空间视图初始化失败，请确认浏览器已启用硬件加速。",
  }));
}

const topCanvas = $("top-canvas");
const frontCanvas = $("front-canvas");

function begin2d(canvas) {
  const ratio = globalThis.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(canvas.clientWidth * ratio));
  const height = Math.max(1, Math.round(canvas.clientHeight * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  return { context, width: canvas.clientWidth, height: canvas.clientHeight };
}

function viewHalfRange(horizontalIndex, verticalIndex) {
  const points = positionTrail.length ? positionTrail : [relativePosition];
  const extent = points.reduce(
    (maximum, point) =>
      Math.max(
        maximum,
        Math.abs(point[horizontalIndex]),
        Math.abs(point[verticalIndex]),
      ),
    0,
  );
  return [0.25, 0.5, 1, 2, 4].find((range) => extent <= range * 0.82) ?? 8;
}

function drawHumanView(canvas, options) {
  const { context, width, height } = begin2d(canvas);
  const margin = Math.min(64, width * 0.17, height * 0.2);
  const halfRange = viewHalfRange(
    options.horizontalIndex,
    options.verticalIndex,
  );
  const scale = Math.min(width - margin * 2, height - margin * 2) /
    (halfRange * 2);
  const center = [width / 2, height / 2];
  const toScreen = (point) => [
    center[0] + point[options.horizontalIndex] * scale,
    center[1] +
    point[options.verticalIndex] * scale * options.verticalScreenSign,
  ];

  context.strokeStyle = "rgba(255,255,255,.07)";
  context.lineWidth = 1;
  for (let index = -4; index <= 4; index++) {
    const offset = index * Math.min(width - margin * 2, height - margin * 2) /
      8;
    context.beginPath();
    context.moveTo(center[0] + offset, margin);
    context.lineTo(center[0] + offset, height - margin);
    context.moveTo(margin, center[1] + offset);
    context.lineTo(width - margin, center[1] + offset);
    context.stroke();
  }
  context.strokeStyle = "rgba(255,255,255,.32)";
  context.lineWidth = 2;
  context.beginPath();
  context.moveTo(margin, center[1]);
  context.lineTo(width - margin, center[1]);
  context.moveTo(center[0], margin);
  context.lineTo(center[0], height - margin);
  context.stroke();

  context.font = "700 13px Inter, system-ui, sans-serif";
  context.fillStyle = "#ff8f93";
  context.textBaseline = "middle";
  context.textAlign = "right";
  context.fillText(options.rightLabel, width - margin - 9, center[1] - 12);
  context.textAlign = "left";
  context.fillText(options.leftLabel, margin + 9, center[1] - 12);
  context.fillStyle = options.verticalColor;
  context.textAlign = "center";
  context.textBaseline = "top";
  context.fillText(options.topLabel, center[0] + 12, margin + 9);
  context.textBaseline = "bottom";
  context.fillText(options.bottomLabel, center[0] + 12, height - margin - 9);
  context.font = "600 11px ui-monospace, monospace";
  context.fillStyle = "rgba(255,255,255,.55)";
  context.textAlign = "left";
  context.fillText(`范围 ±${Math.round(halfRange * 100)} 厘米`, 14, 13);

  if (positionFrame && positionTrail.length > 1) {
    context.strokeStyle = "rgba(75,216,220,.45)";
    context.lineWidth = 2;
    context.beginPath();
    positionTrail.forEach((point, index) => {
      const screen = toScreen(point);
      index ? context.lineTo(...screen) : context.moveTo(...screen);
    });
    context.stroke();
  }
  const current = toScreen(relativePosition);
  if (reference) {
    context.setLineDash([6, 6]);
    context.strokeStyle = "rgba(255,189,102,.62)";
    context.beginPath();
    context.moveTo(...center);
    context.lineTo(...current);
    context.stroke();
    context.setLineDash([]);
  }
  context.fillStyle = "#e8edf2";
  context.beginPath();
  context.arc(...center, 5, 0, Math.PI * 2);
  context.fill();
  context.font = "600 11px Inter, system-ui, sans-serif";
  context.fillStyle = "rgba(232,237,242,.72)";
  context.textAlign = "left";
  context.fillText("标定原点", center[0] + 9, center[1] + 8);
  if (reference) {
    context.shadowColor = "rgba(75,216,220,.75)";
    context.shadowBlur = 16;
    context.fillStyle = "#4bd8dc";
    context.beginPath();
    context.arc(...current, 8, 0, Math.PI * 2);
    context.fill();
    context.shadowBlur = 0;
    context.fillStyle = "#c9f8fa";
    context.fillText(
      SOURCE_LABELS[selectedSource],
      current[0] + 11,
      current[1] - 8,
    );
  }
}

function drawPositionViews() {
  drawHumanView(topCanvas, {
    horizontalIndex: 0,
    verticalIndex: 2,
    verticalScreenSign: 1,
    leftLabel: positionFrame ? "左" : "跟踪轴 −X",
    rightLabel: positionFrame ? "右" : "跟踪轴 +X",
    topLabel: positionFrame ? "前" : "跟踪轴 −Z",
    bottomLabel: positionFrame ? "后" : "跟踪轴 +Z",
    verticalColor: "#82c8ff",
  });
  drawHumanView(frontCanvas, {
    horizontalIndex: 0,
    verticalIndex: 1,
    verticalScreenSign: -1,
    leftLabel: positionFrame ? "左" : "跟踪轴 −X",
    rightLabel: positionFrame ? "右" : "跟踪轴 +X",
    topLabel: positionFrame ? "上" : "跟踪轴 +Y",
    bottomLabel: positionFrame ? "下" : "跟踪轴 −Y",
    verticalColor: "#8cf0aa",
  });
  requestAnimationFrame(drawPositionViews);
}
requestAnimationFrame(drawPositionViews);
