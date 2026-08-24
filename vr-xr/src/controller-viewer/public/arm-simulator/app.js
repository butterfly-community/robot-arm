import * as THREE from "https://cdn.jsdelivr.net/npm/three@0.180.0/+esm";
import { OrbitControls } from "https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/controls/OrbitControls.js/+esm";
import { loadStarArmModel } from "./urdf-model.js";

const $ = (id) => document.getElementById(id);
const JOINT_COUNT = 6;
const DISPLAY_JOINT_COUNT = 7;
const POLL_INTERVAL_MS = 33;
const FRAMING_VERTICAL_OFFSET_RATIO = 0.18;
const MODEL_JOINT_LIMITS_DEG = [
  [-110, 110],
  [0, 180],
  [-270, 0],
  [-90, 90],
  [-65, 65],
  [-150, 150],
  [0, 45],
];
const modelJointTargets = new Float64Array(JOINT_COUNT);
const liveModelJointDegrees = new Float64Array(DISPLAY_JOINT_COUNT);
const manualModelJointDegrees = new Float64Array(DISPLAY_JOINT_COUNT);
const jointRows = [];
let robotModel = null;
let lastSnapshotAt = 0;
let latestSnapshot = null;
let pollPending = false;
let fittedOnce = false;
let manualOverride = false;
let visualGripperRad = 0;

const stateLabels = {
  idle: "待机",
  active: "示教仿真中",
  constrained: "本次目标已丢弃，继续采样",
  faulted: "仿真已停止",
};
const stopReasonLabels = {
  intent_idle: "未按下 Squeeze",
  intent_faulted: "手柄输入无效",
  invalid_intent: "输入数据非法",
  workspace_violation: "超出工作空间",
  singularity: "MoveIt：接近奇异位形",
  joint_limit: "MoveIt：达到关节限位",
  servo_unavailable: "MoveIt Servo 未连接",
  servo_feedback_stale: "MoveIt 关节反馈超时",
  servo_invalid_feedback: "MoveIt 关节反馈非法",
  awaiting_intent_release: "反馈恢复后请松开 Squeeze 再接管",
  servo_halt: "MoveIt Servo 已停止",
  servo_collision: "MoveIt：碰撞停止",
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
  const sphere = new THREE.Mesh(
    new THREE.SphereGeometry(size, 20, 12),
    new THREE.MeshBasicMaterial({
      color,
      transparent: opacity < 1,
      opacity,
      depthTest: true,
    }),
  );
  group.add(sphere);
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

function createJointRows() {
  const fragment = document.createDocumentFragment();
  for (let index = 0; index < DISPLAY_JOINT_COUNT; index += 1) {
    const row = document.createElement("div");
    row.className = "joint-row";
    const label = index === JOINT_COUNT ? "J7" : `J${index + 1}`;
    const [minimum, maximum] = MODEL_JOINT_LIMITS_DEG[index];
    row.innerHTML =
      `<strong>${label}</strong><input class="joint-slider" type="range" min="${minimum}" max="${maximum}" step="0.5" value="0" aria-label="${label} 模型角"><span class="joint-values"><b>0.0°</b><small>模型角</small></span>`;
    fragment.append(row);
    const slider = row.querySelector("input");
    jointRows.push({
      slider,
      angle: row.querySelector("b"),
      speed: row.querySelector("small"),
    });
    slider.addEventListener("input", () => {
      if (!manualOverride) {
        manualModelJointDegrees.set(liveModelJointDegrees);
        manualOverride = true;
      }
      manualModelJointDegrees[index] = Number(slider.value);
      applyVisualJointDegrees(manualModelJointDegrees, null);
      updateManualModeUi();
    });
  }
  $("joint-list").append(fragment);
}

function jointValuesText(degrees) {
  return `[${
    Array.from(degrees, (value) => `${value.toFixed(1)}°`).join(", ")
  }]`;
}

function applyVisualJointDegrees(degrees, velocities) {
  for (let index = 0; index < JOINT_COUNT; index += 1) {
    modelJointTargets[index] = THREE.MathUtils.degToRad(degrees[index]);
  }
  visualGripperRad = THREE.MathUtils.degToRad(degrees[JOINT_COUNT]);
  robotModel?.setArmJoints(modelJointTargets);
  robotModel?.setGripper(visualGripperRad);
  degrees.forEach((value, index) => {
    const row = jointRows[index];
    row.slider.value = String(value);
    row.angle.textContent = `${value.toFixed(1)}°`;
    row.speed.textContent = velocities
      ? index < JOINT_COUNT
        ? `${velocities[index].toFixed(3)} rad/s · 模型角`
        : "实时夹爪模型角"
      : "手动模型角";
  });
  $("manual-joint-values").textContent = `模型角 ${jointValuesText(degrees)}`;
}

function updateManualModeUi() {
  $("joint-mode").textContent = manualOverride
    ? "手动调整（仅本地）"
    : "实时跟随";
  $("resume-live").hidden = !manualOverride;
  $("manual-joint-values").classList.toggle("manual", manualOverride);
  if (manualOverride) {
    $("simulation-state").textContent = "手动调整关节";
    $("simulation-mode").textContent = "LOCAL ONLY";
    $("enabled-state").textContent = "未发送";
    $("stop-reason").textContent = "仅改变浏览器模型";
    currentTcpMarker.visible = false;
    desiredTcpMarker.visible = false;
  }
}

function finiteArray(value, length) {
  return Array.isArray(value) && value.length === length &&
    value.every(Number.isFinite);
}

function finitePose(value) {
  return value && finiteArray(value.position_m, 3) &&
    finiteArray(value.orientation_xyzw, 4);
}

function setMarker(marker, pose) {
  marker.visible = finitePose(pose);
  if (!marker.visible) return;
  marker.position.fromArray(pose.position_m);
  const [x, y, z, w] = pose.orientation_xyzw;
  marker.quaternion.set(x, y, z, w).normalize();
}

function vectorText(value, digits = 3) {
  if (!finiteArray(value, 3)) return "—";
  return `[${value.map((item) => item.toFixed(digits)).join(", ")}] m`;
}

function quaternionText(value) {
  if (!finiteArray(value, 4)) return "—";
  return `[${value.map((item) => item.toFixed(4)).join(", ")}]`;
}

function updateSnapshot(snapshot) {
  if (!snapshot || !finiteArray(snapshot.joints_rad, JOINT_COUNT)) {
    throw new Error("latestArmSimulation 缺少有效 joints_rad[6]");
  }
  if (!finiteArray(snapshot.model_joints_rad, JOINT_COUNT)) {
    throw new Error("latestArmSimulation 缺少有效 model_joints_rad[6]");
  }
  if (!Number.isFinite(snapshot.gripper_rad)) {
    throw new Error("latestArmSimulation 缺少有效 gripper_rad");
  }
  latestSnapshot = snapshot;
  snapshot.model_joints_rad.forEach((value, index) => {
    liveModelJointDegrees[index] = THREE.MathUtils.radToDeg(value);
  });
  liveModelJointDegrees[JOINT_COUNT] = THREE.MathUtils.radToDeg(
    snapshot.gripper_rad,
  );
  lastSnapshotAt = performance.now();
  const state = stateLabels[snapshot.state] ?? snapshot.state ?? "未知";
  $("simulation-state").textContent = manualOverride ? "手动调整关节" : state;
  $("model-id").textContent = snapshot.model_id ?? "—";
  $("backend-name").textContent = snapshot.backend === "moveit_servo"
    ? "MoveIt Servo"
    : snapshot.backend ?? "—";
  $("simulation-mode").textContent = manualOverride
    ? "LOCAL ONLY"
    : snapshot.simulation_only
    ? "SIM ONLY"
    : "状态异常";
  $("enabled-state").textContent = manualOverride
    ? "未发送"
    : snapshot.enabled
    ? "已接管"
    : "未接管";
  $("stop-reason").textContent = manualOverride
    ? "仅改变浏览器模型"
    : snapshot.stop_reason
    ? (stopReasonLabels[snapshot.stop_reason] ?? snapshot.stop_reason)
    : "—";
  $("servo-status").textContent = Number.isInteger(snapshot.servo_status_code)
    ? `${snapshot.servo_status_code}: ${snapshot.servo_status_message ?? "—"}`
    : "—";
  $("feedback-age").textContent = Number.isFinite(
      snapshot.servo_feedback_age_ms,
    )
    ? `${snapshot.servo_feedback_age_ms} ms`
    : "—";
  $("gripper-state").textContent = snapshot.gripper_closed
    ? "扳机按下 / 闭合"
    : "扳机松开 / 张开";
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
  const velocities = finiteArray(snapshot.joint_velocity_rad_s, JOINT_COUNT)
    ? snapshot.joint_velocity_rad_s
    : new Array(JOINT_COUNT).fill(0);
  if (manualOverride) {
    currentTcpMarker.visible = false;
    desiredTcpMarker.visible = false;
  } else {
    applyVisualJointDegrees(liveModelJointDegrees, velocities);
  }
  updateManualModeUi();
  const connection = $("connection-state");
  if (manualOverride) {
    connection.textContent = "本地手动调整";
    connection.className = "badge constrained";
  } else if (snapshot.state === "faulted") {
    connection.textContent = "仿真故障";
    connection.className = "badge faulted";
  } else if (snapshot.state === "constrained") {
    connection.textContent = "目标已丢弃";
    connection.className = "badge constrained";
  } else {
    connection.textContent = "实时数据";
    connection.className = "badge";
  }
}

async function pollStatus() {
  if (pollPending) return;
  pollPending = true;
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const payload = await response.json();
    updateSnapshot(payload.latestArmSimulation);
  } catch (error) {
    const connection = $("connection-state");
    connection.textContent = "数据连接失败";
    connection.className = "badge faulted";
    console.error(error);
  } finally {
    pollPending = false;
  }
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
}

function fitModel() {
  if (!robotModel) return;
  const box = new THREE.Box3().setFromObject(robotModel.root);
  if (box.isEmpty()) return;
  const sphere = box.getBoundingSphere(new THREE.Sphere());
  const direction = camera.position.clone().sub(controls.target).normalize();
  const halfFov = THREE.MathUtils.degToRad(camera.fov * 0.5);
  const distance = Math.max(0.35, sphere.radius / Math.sin(halfFov) * 1.2);
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

function animate(now) {
  if (lastSnapshotAt && now - lastSnapshotAt > 1200) {
    const connection = $("connection-state");
    connection.textContent = "数据已暂停";
    connection.className = "badge waiting";
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

createJointRows();
setView("iso");
new ResizeObserver(resize).observe(viewport);
document.querySelectorAll("[data-view]").forEach((button) => {
  button.addEventListener("click", () => setView(button.dataset.view));
});
$("fit-model").addEventListener("click", fitModel);
$("resume-live").addEventListener("click", () => {
  manualOverride = false;
  if (latestSnapshot) updateSnapshot(latestSnapshot);
  else updateManualModeUi();
});

try {
  robotModel = await loadStarArmModel((loaded, total) => {
    $("model-state").textContent = `加载模型 ${loaded}/${total}`;
  });
  scene.add(robotModel.root);
  robotModel.setArmJoints(modelJointTargets);
  robotModel.setGripper(visualGripperRad);
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
