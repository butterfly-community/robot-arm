const publicDirectory = new URL(
  "../src/controller-viewer/public/arm-simulator/",
  import.meta.url,
);

function source(name: string): string {
  return Deno.readTextFileSync(new URL(name, publicDirectory));
}

Deno.test("arm simulator remains an independent viewer", () => {
  const controllerHtml = Deno.readTextFileSync(
    new URL("../src/controller-viewer/public/index.html", import.meta.url),
  );
  const simulatorHtml = source("index.html");
  if (controllerHtml.includes("arm-simulator/app.js")) {
    throw new Error("controller viewer unexpectedly loads the arm simulator");
  }
  if (!simulatorHtml.includes('src="./app.js')) {
    throw new Error("arm simulator has no independent module entry point");
  }
});

Deno.test("manual controls use the unified APIs and URDF limits", () => {
  const app = source("app.js");
  const model = source("urdf-model.js");
  const html = source("index.html");
  if (!app.includes('fetch("/api/status"')) {
    throw new Error("unified status is not consumed");
  }
  for (
    const endpoint of [
      "/api/arm-control-mode",
      "/api/arm-teleop-components",
      "/api/arm-motion",
      "/api/arm-gripper",
      "/api/arm-motion/cancel",
      "/api/arm-serial/connect",
      "/api/arm-serial/disconnect",
    ]
  ) {
    if (!app.includes(endpoint)) throw new Error(`missing ${endpoint}`);
  }
  if (
    !app.includes('slider.addEventListener("input"') ||
    !app.includes('slider.addEventListener("change"') ||
    !app.includes("targetDegrees.slice(0, JOINT_COUNT)") ||
    !app.includes("submitGripperTarget") ||
    !app.includes("THREE.MathUtils.degToRad(GRIPPER_START_POSITION_DEG)")
  ) {
    throw new Error(
      "sliders do not preview then submit the correct request shape",
    );
  }
  if (
    !model.includes("jointLimitsRad") ||
    !model.includes("root.joints[name]?.limit") ||
    app.includes("MODEL_JOINT_LIMITS_DEG") ||
    app.includes('step="0.5"') ||
    !app.includes('step="any"')
  ) {
    throw new Error("slider ranges are not read directly from the loaded URDF");
  }
  if (
    !app.includes("robotModel?.setArmJoints(modelJoints)") ||
    !app.includes("robotModel?.setGripper(snapshot.gripper_rad)") ||
    !html.includes("目标预览")
  ) {
    throw new Error(
      "actual model and target preview are not clearly separated",
    );
  }
  if (app.includes("model_joints_rad") || html.includes("simulation-state")) {
    throw new Error("old simulation-only joint state remains");
  }
});

Deno.test("three backend-owned checkboxes independently select teleop components", () => {
  const app = source("app.js");
  const html = source("index.html");
  for (
    const [id, label, checked] of [
      ["teleop-position", "空间位置移动", true],
      ["teleop-pitch", "前部抬起 / 前部往下", true],
      [
        "teleop-turn",
        "左旋（俯视夹爪尖端逆时针）/ 右旋（顺时针）",
        true,
      ],
    ] as const
  ) {
    const input = `id="${id}" type="checkbox"${checked ? " checked" : ""}`;
    if (!html.includes(input)) {
      throw new Error(`${label} 的默认勾选状态不正确`);
    }
    if (!html.includes(label)) throw new Error(`缺少采集开关：${label}`);
    if (!app.includes(`$("${id}").checked`)) {
      throw new Error(`${label} 没有同步后端状态`);
    }
  }
  if (
    !app.includes("payload.teleopComponents") ||
    !app.includes('"/api/arm-teleop-components"') ||
    !app.includes("while (desiredTeleopComponents !== null)") ||
    !app.includes("if (teleopComponentsSubmitting) return") ||
    app.includes("await withRequest(async () => {\n    teleopComponents") ||
    app.includes("localStorage") ||
    app.includes("sessionStorage")
  ) {
    throw new Error("采集开关没有由 Rust 后端会话统一保存");
  }
});

Deno.test("the signed start pose is an ordinary motion and keeps the gripper separate", () => {
  const app = source("app.js");
  const html = source("index.html");
  if (
    !app.includes("const START_POSITION_DEG = [0, 0, -3, 0, 0, 0]") ||
    !app.includes("targetDegrees.set(START_POSITION_DEG, 0)") ||
    !app.includes("void moveArmToStartPosition()") ||
    !html.includes("J1–J6 起始位") ||
    !html.includes(
      "目标预览 [0.0°, 0.0°, -3.0°, 0.0°, 0.0°, 0.0°, 1.0°]",
    )
  ) {
    throw new Error(
      "start button does not synchronize the signed six-joint pose",
    );
  }
  for (
    const legacy of [
      "arm-home",
      "HomeStatus",
      "latestArmSimulation",
      "latestArmHardware",
      "armOutputBackend",
      "selectOutputBackend",
      "manualOverride",
      "gripper_closed",
    ]
  ) {
    if (app.includes(legacy)) throw new Error(`legacy path remains: ${legacy}`);
  }
});

Deno.test("connected serial state replaces the connect controls with device and command information", () => {
  const app = source("app.js");
  const html = source("index.html");
  if (
    !app.includes('$("serial-connect-controls").hidden = connected') ||
    !app.includes('$("serial-connected-info").hidden = !connected') ||
    !app.includes('$("serial-connected-port").textContent =') ||
    !html.includes('id="serial-port" type="text" value="/dev/ttyUSB0"') ||
    !html.includes('id="serial-connected-info" hidden') ||
    !html.includes("已连接设备") ||
    !html.includes("J1–J6 与夹爪命令发送到舵机 ID 0–6")
  ) {
    throw new Error("串口已连接时仍保留连接按钮或缺少连接信息");
  }
});

Deno.test("servo deceleration is separate from the stop reason", () => {
  const app = source("app.js");
  const html = source("index.html");
  if (
    !html.includes("<dt>停止原因</dt>") ||
    html.includes("提示 / 停止原因") ||
    !html.includes("<dt>Servo 状态</dt>") ||
    !app.includes('singularity: "MoveIt：奇异位停止"')
  ) {
    throw new Error("Servo warning and actual stop reason remain conflated");
  }
});

Deno.test("colors framing and persistent text selection are preserved", () => {
  const app = source("app.js");
  const model = source("urdf-model.js");
  if (
    !model.includes("await geometryLoaded") ||
    !model.includes("materialForLink")
  ) {
    throw new Error("model colors are applied before STL loading completes");
  }
  if (
    !app.includes('document.addEventListener("selectstart"') ||
    !app.includes('document.addEventListener("selectionchange"') ||
    !app.includes("if (textSelectionActive) return") ||
    !source("style.css").includes("user-select: text")
  ) {
    throw new Error("live repaint can still clear selected text");
  }
  if (
    !app.includes("FRAMING_VERTICAL_OFFSET_RATIO = 0.55") ||
    !app.includes("framedTarget.z += sphere.radius") ||
    !app.includes("if (robotModel) fitModel()") ||
    !app.includes("floor.position.z = -0.001")
  ) {
    throw new Error("low framing regression");
  }
});

Deno.test("arm simulator DOM references all resolve", () => {
  const app = source("app.js");
  const html = source("index.html");
  const referencedIds = [...app.matchAll(/\$\("([^"]+)"\)/g)].map((match) =>
    match[1]
  );
  const htmlIds = [...html.matchAll(/\bid="([^"]+)"/g)].map((match) =>
    match[1]
  );
  const missing = [...new Set(referencedIds)].filter((id) =>
    !htmlIds.includes(id)
  );
  const duplicate = htmlIds.filter((id, index) =>
    htmlIds.indexOf(id) !== index
  );
  if (missing.length || duplicate.length) {
    throw new Error(JSON.stringify({ missing, duplicate }));
  }
});

Deno.test("the generated URDF path maps six arm joints and one gripper", () => {
  const model = source("urdf-model.js");
  for (let index = 1; index <= 6; index += 1) {
    if (!model.includes(`"joint${index}"`)) {
      throw new Error(`joint${index} is missing`);
    }
  }
  if (
    !model.includes('"joint7_left"') ||
    !model.includes("./models/stararm102_description.urdf") ||
    !model.includes("root.joints[name]?.limit")
  ) {
    throw new Error("gripper control is missing");
  }
});

Deno.test("the rendered model labels every joint with its servo ID", () => {
  const model = source("urdf-model.js");
  const app = source("app.js");
  const html = source("index.html");
  for (
    const [joint, label] of [
      ["joint1", "J1 / ID 0"],
      ["joint2", "J2 / ID 1"],
      ["joint3", "J3 / ID 2"],
      ["joint4", "J4 / ID 3"],
      ["joint5", "J5 / ID 4"],
      ["joint6", "J6 / ID 5"],
      ["joint7_left", "夹爪 / ID 6"],
    ]
  ) {
    if (!model.includes(`["${joint}", "${label}"]`)) {
      throw new Error(`missing model label ${label}`);
    }
  }
  if (
    !model.includes("joint.add(label.sprite)") ||
    !model.includes("new THREE.Sprite") ||
    !model.includes("setJointLabelsVisible(visible)") ||
    !model.includes("setJointParameters(parameters)") ||
    !model.includes('setParameters("参数未读取")') ||
    !html.includes('id="joint-labels-toggle" type="checkbox" checked') ||
    !html.includes("显示关节 ID / PID") ||
    !app.includes(
      "robotModel?.setJointLabelsVisible(event.currentTarget.checked)",
    ) ||
    !app.includes("latestSerialState.internal_parameters") ||
    !app.includes("robotModel.setJointParameters")
  ) {
    throw new Error(
      "joint labels, visibility switch, or live parameter binding is incomplete",
    );
  }
});

Deno.test("combined dashboard embeds both viewers", () => {
  const dashboard = Deno.readTextFileSync(
    new URL(
      "../src/controller-viewer/public/test-dashboard/index.html",
      import.meta.url,
    ),
  );
  for (const frame of ['src="/"', 'src="/arm-simulator/"']) {
    if (!dashboard.includes(frame)) {
      throw new Error(`dashboard is missing ${frame}`);
    }
  }
});
