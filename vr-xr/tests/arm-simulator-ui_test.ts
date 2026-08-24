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

Deno.test("arm simulator uses pinned CDN modules and the read-only status API", () => {
  const app = source("app.js");
  const model = source("urdf-model.js");
  for (const text of [app, model]) {
    if (!text.includes("https://cdn.jsdelivr.net/npm/three@0.180.0/")) {
      throw new Error("pinned Three.js CDN import is missing");
    }
  }
  if (!app.includes('fetch("/api/status"')) {
    throw new Error("simulator does not read /api/status");
  }
  for (const unsafeMethod of ["POST", "PUT", "PATCH", "DELETE"]) {
    if (app.includes(`method: "${unsafeMethod}"`)) {
      throw new Error(`simulator unexpectedly contains ${unsafeMethod}`);
    }
  }
  if (!app.includes("payload.latestArmSimulation")) {
    throw new Error("simulator does not consume latestArmSimulation");
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
  if (!app.includes('constrained: "本次目标已丢弃，继续采样"')) {
    throw new Error(
      "simulator does not distinguish a recoverable constraint from a fault",
    );
  }
  if (
    !app.includes('awaiting_intent_release: "反馈恢复后请松开 Squeeze 再接管"')
  ) {
    throw new Error(
      "simulator does not explain the post-fault rearm requirement",
    );
  }
  if (
    !app.includes("FRAMING_VERTICAL_OFFSET_RATIO = 0.18") ||
    !app.includes("framedTarget.z += sphere.radius")
  ) {
    throw new Error(
      "simulator does not keep the ground plane near the vertical center",
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
