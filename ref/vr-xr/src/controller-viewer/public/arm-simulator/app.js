import * as THREE from "https://esm.sh/three@0.180.0?target=es2022";
import { OrbitControls } from "https://esm.sh/three@0.180.0/examples/jsm/controls/OrbitControls.js?target=es2022";
import { loadStarArmModel } from "./urdf-model.js?v=20260826-6";

const $ = (id) => document.getElementById(id);
const JOINT_COUNT = 6;
const CONTROL_COUNT = 7;
const START_POSITION_DEG = [0, 0, -3, 0, 0, 0];
const GRIPPER_START_POSITION_DEG = 1;
const POLL_INTERVAL_MS = 33;
const FRAMING_VERTICAL_OFFSET_RATIO = 0.55;
const liveDegrees = new Float64Array(CONTROL_COUNT);
const targetDegrees = new Float64Array(CONTROL_COUNT);
const modelJoints = new Float64Array(JOINT_COUNT);
liveDegrees.set([...START_POSITION_DEG, GRIPPER_START_POSITION_DEG]);
targetDegrees.set(liveDegrees);
modelJoints.set(START_POSITION_DEG.map(THREE.MathUtils.degToRad));
const jointRows = [];
let robotModel = null;
let latestSnapshot = null;
let latestMotionStatus = null;
let latestSerialState = null;
let controlMode = "teleop";
let teleopComponents = { position: true, pitch: true, turn: true };
let desiredTeleopComponents = null;
let teleopComponentsSubmitting = false;
let manualInitialized = false;
let previewIndex = null;
let requestPending = false;
let pollPending = false;
let fittedOnce = false;
let textSelectionActive = false;
let textSelectionPointerDown = false;
let jointParameterSignature;

const stateLabels = {
  idle: "待机",
  active: "示教中",
  constrained: "MoveIt 正在约束运动",
  faulted: "输出已停止",
};
const stopReasonLabels = {
  intent_idle: "未按下 Trigger",
  intent_faulted: "手柄输入无效",
  invalid_intent: "输入数据非法",
  singularity: "MoveIt：奇异位停止",
  joint_limit: "MoveIt：达到关节限位",
  servo_unavailable: "MoveIt Servo 未连接",
  servo_invalid_feedback: "MoveIt 关节反馈非法",
  servo_collision: "MoveIt：碰撞停止",
};
const motionStateLabels = {
  idle: "尚未请求",
  planning: "正在规划",
  executing: "正在执行",
  succeeded: "执行完成",
  failed: "执行失败",
  cancelled: "已取消",
};

const canvas = $("robot-canvas");
const viewport = $("viewport");
const renderer = new THREE.WebGLRenderer({
  canvas,
  antialias: true,
  powerPreference: "high-performance",
});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x08121b);
scene.fog = new THREE.Fog(0x08121b, 1.4, 2.6);
const camera = new THREE.PerspectiveCamera(38, 1, 0.005, 5);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.07;
controls.minDistance = 0.18;
controls.maxDistance = 2.2;
controls.target.set(0.1, 0, 0.15);
scene.add(new THREE.HemisphereLight(0xccecff, 0x18222b, 2.2));
const keyLight = new THREE.DirectionalLight(0xffffff, 3.1);
keyLight.position.set(0.45, -0.5, 0.85);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(2048, 2048);
keyLight.shadow.camera.near = 0.05;
keyLight.shadow.camera.far = 2;
keyLight.shadow.camera.left = -0.55;
keyLight.shadow.camera.right = 0.55;
keyLight.shadow.camera.top = 0.55;
keyLight.shadow.camera.bottom = -0.55;
scene.add(keyLight);
const fillLight = new THREE.DirectionalLight(0x68b9e8, 1.2);
fillLight.position.set(-0.5, 0.35, 0.4);
scene.add(fillLight);

const floor = new THREE.Mesh(
  new THREE.CircleGeometry(0.62, 96),
  new THREE.MeshStandardMaterial({
    color: 0x0c1b25,
    roughness: 0.92,
    metalness: 0.08,
  }),
);
floor.position.z = -0.001;
floor.receiveShadow = true;
scene.add(floor);
const grid = new THREE.GridHelper(1.1, 22, 0x35758d, 0x1b3a49);
grid.rotation.x = Math.PI / 2;
grid.position.z = 0.0005;
grid.material.opacity = 0.42;
grid.material.transparent = true;
scene.add(grid);
scene.add(new THREE.AxesHelper(0.13));

function poseMarker(color, opacity, size) {
  const group = new THREE.Group();
  group.add(
    new THREE.Mesh(
      new THREE.SphereGeometry(size, 20, 12),
      new THREE.MeshBasicMaterial({
        color,
        transparent: opacity < 1,
        opacity,
        depthTest: true,
      }),
    ),
  );
  const axes = new THREE.AxesHelper(size * 4.5);
  axes.material.transparent = opacity < 1;
  axes.material.opacity = opacity;
  group.add(axes);
  group.visible = false;
  scene.add(group);
  return group;
}

const currentTcpMarker = poseMarker(0xff9b5a, 1, 0.009);
const desiredTcpMarker = poseMarker(0x47d8eb, 0.7, 0.007);

function finiteArray(value, length) {
  return Array.isArray(value) && value.length === length &&
    value.every(Number.isFinite);
}

function jointValuesText(values) {
  return `[${
    Array.from(values, (value) => `${value.toFixed(1)}°`).join(", ")
  }]`;
}

function updateTargetText() {
  const label = previewIndex === null
    ? "手动目标"
    : `目标预览 ${
      previewIndex === JOINT_COUNT ? "夹爪" : `J${previewIndex + 1}`
    }`;
  $("manual-joint-values").textContent = `${label} ${
    jointValuesText(targetDegrees)
  }`;
  $("manual-joint-values").classList.toggle("manual", previewIndex !== null);
}

function syncTargetRows() {
  jointRows.forEach((row, index) => {
    row.slider.value = String(targetDegrees[index]);
    row.angle.textContent = `${targetDegrees[index].toFixed(1)}°`;
    row.detail.textContent = "手动目标";
  });
  updateTargetText();
}

function initializeManualTargets() {
  targetDegrees.set(liveDegrees);
  manualInitialized = true;
  previewIndex = null;
  syncTargetRows();
}

function createJointRows(limitsRad) {
  const fragment = document.createDocumentFragment();
  limitsRad.forEach(([minimum, maximum], index) => {
    const row = document.createElement("div");
    row.className = "joint-row";
    const label = index === JOINT_COUNT ? "夹爪" : `J${index + 1}`;
    row.innerHTML =
      `<strong>${label}</strong><input class="joint-slider" type="range" min="${
        THREE.MathUtils.radToDeg(minimum)
      }" max="${THREE.MathUtils.radToDeg(maximum)}" step="any" value="${
        targetDegrees[index]
      }" aria-label="${label} 目标角"><span class="joint-values"><b>${
        targetDegrees[index].toFixed(1)
      }°</b><small>实时反馈</small></span>`;
    const slider = row.querySelector("input");
    const entry = {
      slider,
      angle: row.querySelector("b"),
      detail: row.querySelector("small"),
    };
    jointRows.push(entry);
    slider.addEventListener("input", () => {
      if (controlMode !== "manual") return;
      previewIndex = index;
      targetDegrees[index] = Number(slider.value);
      entry.angle.textContent = `${targetDegrees[index].toFixed(1)}°`;
      entry.detail.textContent = "目标预览";
      updateTargetText();
    });
    slider.addEventListener("change", () => {
      if (controlMode !== "manual") return;
      targetDegrees[index] = Number(slider.value);
      previewIndex = null;
      syncTargetRows();
      if (index === JOINT_COUNT) {
        void submitGripperTarget(
          THREE.MathUtils.degToRad(targetDegrees[index]),
        );
      } else {
        void submitArmTarget(Array.from(
          targetDegrees.slice(0, JOINT_COUNT),
          THREE.MathUtils.degToRad,
        ));
      }
    });
    fragment.append(row);
  });
  $("joint-list").append(fragment);
  updateControlModeUi();
}

function applyActualModel(snapshot) {
  snapshot.joints_rad.forEach((value, index) => {
    liveDegrees[index] = THREE.MathUtils.radToDeg(value);
    modelJoints[index] = value;
  });
  liveDegrees[JOINT_COUNT] = THREE.MathUtils.radToDeg(snapshot.gripper_rad);
  robotModel?.setArmJoints(modelJoints);
  robotModel?.setGripper(snapshot.gripper_rad);
  if (controlMode !== "manual") {
    targetDegrees.set(liveDegrees);
    jointRows.forEach((row, index) => {
      row.slider.value = String(liveDegrees[index]);
      row.angle.textContent = `${liveDegrees[index].toFixed(1)}°`;
      row.detail.textContent = "实时反馈";
    });
    updateTargetText();
  }
}

function finitePose(value) {
  return value && finiteArray(value.position_m, 3) &&
    finiteArray(value.orientation_xyzw, 4);
}

function setMarker(marker, pose) {
  marker.visible = finitePose(pose);
  if (!marker.visible) return;
  marker.position.fromArray(pose.position_m);
  marker.quaternion.fromArray(pose.orientation_xyzw).normalize();
}

function vectorText(value, digits = 3) {
  return finiteArray(value, 3)
    ? `[${value.map((item) => item.toFixed(digits)).join(", ")}] m`
    : "—";
}

function quaternionText(value) {
  return finiteArray(value, 4)
    ? `[${value.map((item) => item.toFixed(4)).join(", ")}]`
    : "—";
}

function updateSnapshot(snapshot) {
  if (
    !snapshot || !finiteArray(snapshot.joints_rad, JOINT_COUNT) ||
    !Number.isFinite(snapshot.gripper_rad)
  ) {
    throw new Error("latestArmState 缺少有效的 J1–J6 或夹爪反馈");
  }
  latestSnapshot = snapshot;
  applyActualModel(snapshot);
  $("arm-state").textContent = stateLabels[snapshot.state] ??
    snapshot.state ?? "未知";
  $("model-id").textContent = snapshot.model_id ?? "—";
  const serialFeedback = snapshot.feedback_source === "serial";
  $("feedback-source").textContent = serialFeedback
    ? "串口 Monitor"
    : "ROS 软件反馈";
  $("feedback-mode").textContent = serialFeedback ? "SERIAL" : "SOFTWARE";
  $("enabled-state").textContent = snapshot.enabled ? "已接管" : "未接管";
  $("stop-reason").textContent = snapshot.stop_reason
    ? (stopReasonLabels[snapshot.stop_reason] ?? snapshot.stop_reason)
    : "—";
  $("servo-status").textContent = Number.isInteger(snapshot.servo_status_code)
    ? `${snapshot.servo_status_code}: ${snapshot.servo_status_message ?? "—"}`
    : "—";
  $("feedback-age").textContent =
    Number.isFinite(snapshot.servo_feedback_age_ms)
      ? `${snapshot.servo_feedback_age_ms} ms`
      : "—";
  $("gripper-angle").textContent = `${
    THREE.MathUtils.radToDeg(snapshot.gripper_rad).toFixed(1)
  }°`;
  $("tcp-position").textContent = vectorText(snapshot.tcp_pose?.position_m);
  $("tcp-orientation").textContent = quaternionText(
    snapshot.tcp_pose?.orientation_xyzw,
  );
  $("desired-position").textContent = vectorText(
    snapshot.desired_tcp_pose?.position_m,
  );
  $("desired-orientation").textContent = quaternionText(
    snapshot.desired_tcp_pose?.orientation_xyzw,
  );
  setMarker(currentTcpMarker, snapshot.tcp_pose);
  setMarker(desiredTcpMarker, snapshot.desired_tcp_pose);
  const connection = $("connection-state");
  if (snapshot.state === "faulted") {
    connection.textContent = "控制输出停止";
    connection.className = "badge faulted";
  } else if (snapshot.state === "constrained") {
    connection.textContent = "MoveIt 受约束";
    connection.className = "badge constrained";
  } else {
    connection.textContent = serialFeedback ? "真机反馈" : "软件反馈";
    connection.className = "badge";
  }
  $("output-warning").textContent = serialFeedback
    ? "串口 Monitor 反馈"
    : "软件反馈";
}

function updateMotionStatus(status) {
  latestMotionStatus = status ?? {
    request_id: 0,
    state: "idle",
    message: "尚未请求普通运动",
    trajectory_points: 0,
    duration_seconds: null,
  };
  const busy = ["planning", "executing"].includes(latestMotionStatus.state);
  $("motion-state").textContent = motionStateLabels[latestMotionStatus.state] ??
    latestMotionStatus.state;
  $("motion-message").textContent = latestMotionStatus.message || "—";
  $("motion-points").textContent = latestMotionStatus.trajectory_points > 0
    ? String(latestMotionStatus.trajectory_points)
    : "—";
  $("motion-duration").textContent =
    Number.isFinite(latestMotionStatus.duration_seconds)
      ? `${latestMotionStatus.duration_seconds.toFixed(1)} 秒`
      : "—";
  $("start-position-arm").disabled = requestPending || busy;
  $("cancel-motion").hidden = !busy;
  $("cancel-motion").disabled = requestPending;
}

function updateSerialState(state) {
  latestSerialState = state ??
    { selected_port: null, connected: false, error: null };
  const connected = latestSerialState.connected === true;
  $("serial-state").textContent = connected ? "已连接" : "未连接";
  $("serial-error").textContent = latestSerialState.error ??
    latestSerialState.parameter_error ?? "—";
  const parameterSignature = JSON.stringify(
    latestSerialState.internal_parameters ?? null,
  );
  if (robotModel && parameterSignature !== jointParameterSignature) {
    robotModel.setJointParameters(latestSerialState.internal_parameters);
    jointParameterSignature = parameterSignature;
  }
  if (
    latestSerialState.selected_port &&
    document.activeElement !== $("serial-port")
  ) {
    $("serial-port").value = latestSerialState.selected_port;
  }
  $("serial-connect-controls").hidden = connected;
  $("serial-connected-info").hidden = !connected;
  $("serial-connected-port").textContent = latestSerialState.selected_port ??
    "—";
  $("serial-connect").disabled = requestPending;
  $("serial-disconnect").disabled = requestPending;
}

function updateControlModeUi() {
  const manual = controlMode === "manual";
  $("mode-teleop").setAttribute("aria-pressed", String(!manual));
  $("mode-manual").setAttribute("aria-pressed", String(manual));
  $("mode-teleop").disabled = requestPending;
  $("mode-manual").disabled = requestPending;
  jointRows.forEach((row) => row.slider.disabled = !manual || requestPending);
  $("joint-help").textContent = manual
    ? "拖动只更新目标预览和数值；松开 J1–J6 时提交完整六轴目标，松开夹爪时只提交夹爪角。"
    : "手柄控制中；切换模式本身不产生运动。";
}

function updateTeleopComponentsUi() {
  $("teleop-position").checked = teleopComponents.position;
  $("teleop-pitch").checked = teleopComponents.pitch;
  $("teleop-turn").checked = teleopComponents.turn;
  for (
    const id of [
      "teleop-position",
      "teleop-pitch",
      "teleop-turn",
    ]
  ) {
    $(id).disabled = false;
  }
}

async function requestJson(url, body) {
  const response = await fetch(url, {
    method: "POST",
    cache: "no-store",
    headers: body === undefined
      ? undefined
      : { "content-type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error ?? `HTTP ${response.status}`);
  return payload;
}

async function withRequest(task, errorTarget = "motion-message") {
  if (requestPending) return;
  requestPending = true;
  updateControlModeUi();
  updateTeleopComponentsUi();
  updateMotionStatus(latestMotionStatus);
  updateSerialState(latestSerialState);
  try {
    await task();
    await pollStatus();
  } catch (error) {
    $(errorTarget).textContent = error.message;
    console.error(error);
  } finally {
    requestPending = false;
    updateControlModeUi();
    updateTeleopComponentsUi();
    updateMotionStatus(latestMotionStatus);
    updateSerialState(latestSerialState);
  }
}

async function setControlMode(mode) {
  if (mode === controlMode) return;
  await withRequest(async () => {
    await requestJson("/api/arm-control-mode", { mode });
    controlMode = mode;
    if (mode === "manual") initializeManualTargets();
    else manualInitialized = false;
  });
}

function setTeleopComponents() {
  desiredTeleopComponents = {
    position: $("teleop-position").checked,
    pitch: $("teleop-pitch").checked,
    turn: $("teleop-turn").checked,
  };
  teleopComponents = desiredTeleopComponents;
  updateTeleopComponentsUi();
  void submitTeleopComponents();
}

async function submitTeleopComponents() {
  if (teleopComponentsSubmitting) return;
  teleopComponentsSubmitting = true;
  try {
    while (desiredTeleopComponents !== null) {
      const components = desiredTeleopComponents;
      desiredTeleopComponents = null;
      const applied = await requestJson(
        "/api/arm-teleop-components",
        components,
      );
      if (desiredTeleopComponents === null) {
        teleopComponents = applied;
        updateTeleopComponentsUi();
      }
    }
    $("teleop-components-message").textContent =
      "未勾选的分量不进入机械臂目标。";
  } catch (error) {
    desiredTeleopComponents = null;
    $("teleop-components-message").textContent = error.message;
    console.error(error);
    await pollStatus();
  } finally {
    teleopComponentsSubmitting = false;
    if (desiredTeleopComponents !== null) void submitTeleopComponents();
  }
}

async function submitArmTarget(jointsRad) {
  await withRequest(() =>
    requestJson("/api/arm-motion", { joints_rad: jointsRad })
  );
}

async function submitGripperTarget(positionRad) {
  await withRequest(() =>
    requestJson("/api/arm-gripper", { position_rad: positionRad })
  );
}

async function pollStatus() {
  if (pollPending) return;
  pollPending = true;
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const payload = await response.json();
    latestSnapshot = payload.latestArmState;
    latestMotionStatus = payload.motionStatus;
    latestSerialState = payload.serialState;
    const serverTeleopComponents = payload.teleopComponents ?? {
      position: true,
      pitch: true,
      turn: true,
    };
    if (!teleopComponentsSubmitting && desiredTeleopComponents === null) {
      teleopComponents = serverTeleopComponents;
    }
    const nextMode = payload.controlMode === "manual" ? "manual" : "teleop";
    if (textSelectionActive) return;
    const modeChanged = nextMode !== controlMode;
    controlMode = nextMode;
    updateSnapshot(latestSnapshot);
    if (controlMode === "manual" && (modeChanged || !manualInitialized)) {
      initializeManualTargets();
    }
    if (controlMode !== "manual") manualInitialized = false;
    updateControlModeUi();
    updateTeleopComponentsUi();
    updateMotionStatus(latestMotionStatus);
    updateSerialState(latestSerialState);
  } catch (error) {
    $("connection-state").textContent = "数据连接失败";
    $("connection-state").className = "badge faulted";
    console.error(error);
  } finally {
    pollPending = false;
  }
}

function selectedTextExists() {
  const selection = globalThis.getSelection();
  return Boolean(selection && !selection.isCollapsed && selection.toString());
}

function refreshAfterTextSelection() {
  if (latestSnapshot) updateSnapshot(latestSnapshot);
  updateControlModeUi();
  updateTeleopComponentsUi();
  updateMotionStatus(latestMotionStatus);
  updateSerialState(latestSerialState);
}

function finishTextSelection() {
  const remainsSelected = selectedTextExists();
  const wasActive = textSelectionActive;
  textSelectionActive = remainsSelected;
  if (wasActive && !remainsSelected) refreshAfterTextSelection();
}

function setView(name) {
  const target = new THREE.Vector3(0.1, 0, 0.2);
  const positions = {
    iso: new THREE.Vector3(0.62, -0.62, 0.48),
    front: new THREE.Vector3(0.1, -0.82, 0.2),
    side: new THREE.Vector3(0.82, 0, 0.2),
    top: new THREE.Vector3(0.1, 0, 0.92),
  };
  controls.target.copy(target);
  camera.position.copy(positions[name] ?? positions.iso);
  camera.up.set(0, name === "top" ? 1 : 0, name === "top" ? 0 : 1);
  controls.update();
  if (robotModel) fitModel();
}

function fitModel() {
  if (!robotModel) return;
  const box = new THREE.Box3().setFromObject(robotModel.root);
  if (box.isEmpty()) return;
  const sphere = box.getBoundingSphere(new THREE.Sphere());
  const direction = camera.position.clone().sub(controls.target).normalize();
  const distance = Math.max(
    0.35,
    sphere.radius / Math.sin(THREE.MathUtils.degToRad(camera.fov * 0.5)) * 1.2,
  );
  const framedTarget = sphere.center.clone();
  framedTarget.z += sphere.radius * FRAMING_VERTICAL_OFFSET_RATIO;
  controls.target.copy(framedTarget);
  camera.position.copy(framedTarget).addScaledVector(direction, distance);
  camera.near = Math.max(0.002, distance - sphere.radius * 2.5);
  camera.far = distance + sphere.radius * 6;
  camera.updateProjectionMatrix();
  controls.update();
}

function resize() {
  const width = Math.max(1, viewport.clientWidth);
  const height = Math.max(1, viewport.clientHeight);
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

function animate() {
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

async function moveArmToStartPosition() {
  targetDegrees.set(START_POSITION_DEG, 0);
  previewIndex = null;
  syncTargetRows();
  await submitArmTarget(
    START_POSITION_DEG.map(THREE.MathUtils.degToRad),
  );
}

setView("iso");
new ResizeObserver(resize).observe(viewport);
document.addEventListener("selectstart", () => textSelectionActive = true);
document.addEventListener("selectionchange", () => {
  if (selectedTextExists()) textSelectionActive = true;
  else if (!textSelectionPointerDown) finishTextSelection();
});
document.addEventListener("pointerdown", (event) => {
  textSelectionPointerDown = !event.target.closest("button, a, input, canvas");
  if (textSelectionPointerDown) textSelectionActive = true;
});
globalThis.addEventListener("pointerup", () => {
  textSelectionPointerDown = false;
  requestAnimationFrame(finishTextSelection);
});
document.querySelectorAll("[data-view]").forEach((button) => {
  button.addEventListener("click", () => setView(button.dataset.view));
});
$("fit-model").addEventListener("click", fitModel);
$("joint-labels-toggle").addEventListener("change", (event) => {
  robotModel?.setJointLabelsVisible(event.currentTarget.checked);
});
$("mode-teleop").addEventListener("click", () => void setControlMode("teleop"));
$("mode-manual").addEventListener("click", () => void setControlMode("manual"));
for (
  const id of [
    "teleop-position",
    "teleop-pitch",
    "teleop-turn",
  ]
) {
  $(id).addEventListener("change", setTeleopComponents);
}
$("start-position-arm").addEventListener(
  "click",
  () => void moveArmToStartPosition(),
);
$("cancel-motion").addEventListener("click", () => {
  void withRequest(() => requestJson("/api/arm-motion/cancel"));
});
$("serial-connect").addEventListener("click", () => {
  void withRequest(
    () =>
      requestJson("/api/arm-serial/connect", { port: $("serial-port").value }),
    "serial-error",
  );
});
$("serial-disconnect").addEventListener("click", () => {
  void withRequest(
    () => requestJson("/api/arm-serial/disconnect"),
    "serial-error",
  );
});

try {
  robotModel = await loadStarArmModel((loaded, total) => {
    $("model-state").textContent = `加载模型 ${loaded}/${total}`;
  });
  scene.add(robotModel.root);
  createJointRows(robotModel.jointLimitsRad);
  robotModel.setArmJoints(modelJoints);
  robotModel.setGripper(
    THREE.MathUtils.degToRad(GRIPPER_START_POSITION_DEG),
  );
  robotModel.setJointLabelsVisible($("joint-labels-toggle").checked);
  updateSerialState(latestSerialState);
  $("model-state").textContent = `${robotModel.meshCount} 个 STL · 已就绪`;
  if (!fittedOnce) {
    fittedOnce = true;
    fitModel();
  }
} catch (error) {
  $("model-state").textContent = "模型加载失败";
  $("model-error").textContent = error.message;
  $("model-error").hidden = false;
  console.error(error);
}

resize();
await pollStatus();
setInterval(pollStatus, POLL_INTERVAL_MS);
requestAnimationFrame(animate);
