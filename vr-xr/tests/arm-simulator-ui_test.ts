const publicDirectory = new URL(
  "../src/controller-viewer/public/arm-simulator/",
  import.meta.url,
);

function source(name: string): string {
  return Deno.readTextFileSync(new URL(name, publicDirectory));
}

Deno.test("arm simulator is independent from the existing controller viewer", () => {
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

Deno.test("arm simulator uses pinned CDN modules and the direct output selector", () => {
  const app = source("app.js");
  const model = source("urdf-model.js");
  for (const text of [app, model]) {
    if (!text.includes("https://esm.sh/three@0.180.0")) {
      throw new Error("pinned Three.js CDN import is missing");
    }
  }
  if (!model.includes("urdf-loader@0.13.1?deps=three@0.180.0")) {
    throw new Error("URDF loader or its Three.js peer is not pinned");
  }
  if (!app.includes('fetch("/api/status"')) {
    throw new Error("simulator does not read /api/status");
  }
  if (
    !app.includes("fetch(`/api/arm-output/${backend}`") ||
    !app.includes('method: "POST"')
  ) {
    throw new Error("simulator does not use the dedicated backend selector");
  }
  if (
    !app.includes("`/api/arm-home/${selectedOutputBackend}/${action}`") ||
    !app.includes('void requestHome("plan")') ||
    app.includes('requestHome("execute")') ||
    app.includes("globalThis.confirm")
  ) {
    throw new Error(
      "simulator does not use the single-step home path",
    );
  }
  if (
    !app.includes("payload.armHomeSimulation") ||
    !app.includes("payload.armHomeHardware") ||
    !app.includes("trajectory_model_joints_rad") ||
    !app.includes("showHomePreviewPoint(0, false)")
  ) {
    throw new Error(
      "simulator does not expose the planned home trajectory preview",
    );
  }
  if (!model.includes("await geometryLoaded")) {
    throw new Error(
      "model colors are applied before asynchronous STL loading completes",
    );
  }
  if (
    !app.includes('document.addEventListener("selectstart"') ||
    !app.includes('document.addEventListener("selectionchange"') ||
    !app.includes("if (textSelectionActive) return") ||
    !source("style.css").includes("user-select: text")
  ) {
    throw new Error("live status repaint still clears selected text");
  }
  for (const unsafeMethod of ["PUT", "PATCH", "DELETE"]) {
    if (app.includes(`method: "${unsafeMethod}"`)) {
      throw new Error(`simulator unexpectedly contains ${unsafeMethod}`);
    }
  }
  if (!app.includes("payload.latestArmSimulation")) {
    throw new Error("simulator does not consume latestArmSimulation");
  }
  if (
    !app.includes("payload.latestArmHardware") ||
    !app.includes("payload.armOutputBackend")
  ) {
    throw new Error("simulator does not expose the selected hardware twin");
  }
  if (!app.includes("snapshot.gripper_rad") || !model.includes("setGripper")) {
    throw new Error("simulator does not consume the J7 gripper snapshot");
  }
  if (
    !app.includes('type="range"') ||
    !app.includes("manualOverride = true") ||
    !app.includes('$("resume-live")')
  ) {
    throw new Error("simulator does not provide local-only J1-J7 adjustment");
  }
  if (!app.includes("snapshot.model_joints_rad")) {
    throw new Error("simulator feeds logical angles directly into the URDF");
  }
  if (!app.includes('constrained: "MoveIt 正在约束运动"')) {
    throw new Error(
      "simulator does not distinguish a recoverable constraint from a fault",
    );
  }
  if (app.includes("awaiting_intent_release")) {
    throw new Error(
      "simulator still exposes the removed release-to-rearm path",
    );
  }
  if (
    !app.includes("FRAMING_VERTICAL_OFFSET_RATIO = 0.55") ||
    !app.includes("framedTarget.z += sphere.radius") ||
    !app.includes("if (robotModel) fitModel()")
  ) {
    throw new Error(
      "simulator does not keep the model and ground plane low in every view",
    );
  }
  if (!app.includes("floor.position.z = -0.001")) {
    throw new Error(
      "camera framing unexpectedly changed the physical floor height",
    );
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

Deno.test("arm simulator maps all six simulated joints and manufacturer meshes", () => {
  const model = source("urdf-model.js");
  const urdf = source("models/stararm102_description.urdf");
  for (let index = 1; index <= 6; index += 1) {
    if (!model.includes(`"joint${index}"`)) {
      throw new Error(`joint${index} is missing from simulator mapping`);
    }
    if (!urdf.includes(`name="joint${index}"`)) {
      throw new Error(`joint${index} is missing from copied URDF`);
    }
  }
  for (
    const mesh of [
      "base_link.STL",
      "link1.STL",
      "link2.STL",
      "link3.STL",
      "link4.STL",
      "link5.STL",
      "link6.STL",
      "link7_left.STL",
      "link7_right.STL",
    ]
  ) {
    const info = Deno.statSync(
      new URL(`models/meshes/${mesh}`, publicDirectory),
    );
    if (!info.isFile || info.size < 1000) {
      throw new Error(`${mesh} is missing or truncated`);
    }
  }
});

Deno.test("arm simulator URDF uses confirmed FL limits and one active gripper joint", () => {
  const app = source("app.js");
  const modelReadme = source("models/README.md");
  const urdf = source("models/stararm102_description.urdf");
  if (
    !modelReadme.includes(
      "5979b346eb3a417840b29b76740754e4005d071a",
    )
  ) {
    throw new Error(
      "visualization model source commit is not pinned correctly",
    );
  }
  const limits = [
    ["joint1", "-1.9198621772", "1.9198621772", "[-110, 110]"],
    ["joint2", "0", "3.1415926536", "[0, 180]"],
    ["joint3", "-4.7123889804", "0", "[-270, 0]"],
    ["joint4", "-1.5707963268", "1.5707963268", "[-90, 90]"],
    ["joint5", "-1.1344640138", "1.1344640138", "[-65, 65]"],
    ["joint6", "-2.6179938780", "2.6179938780", "[-150, 150]"],
    ["joint7_left", "0", "1.5707963268", "[0, 90]"],
  ];
  for (const [name, lower, upper, slider] of limits) {
    const block = urdf.match(
      new RegExp(`<joint\\s+name="${name}"[\\s\\S]*?</joint>`),
    )?.[0];
    if (
      !block?.includes(`lower="${lower}"`) ||
      !block.includes(`upper="${upper}"`)
    ) {
      throw new Error(`${name} does not use the confirmed FL model range`);
    }
    if (!app.includes(slider)) {
      throw new Error(`${name} slider does not match the confirmed FL range`);
    }
  }
  const right = urdf.match(
    /<joint\s+name="joint7_right"[\s\S]*?<\/joint>/,
  )?.[0];
  if (
    !right?.includes('<mimic joint="joint7_left" multiplier="-1" />') ||
    !right.includes('lower="-1.5707963268"') ||
    !right.includes('upper="0"')
  ) {
    throw new Error("joint7_right is not the confirmed inverse mimic joint");
  }
});

Deno.test("combined test dashboard embeds both independent viewers", () => {
  const dashboard = Deno.readTextFileSync(
    new URL(
      "../src/controller-viewer/public/test-dashboard/index.html",
      import.meta.url,
    ),
  );
  for (const source of ['src="/"', 'src="/arm-simulator/"']) {
    if (!dashboard.includes(source)) {
      throw new Error(`combined dashboard is missing ${source}`);
    }
  }
});
