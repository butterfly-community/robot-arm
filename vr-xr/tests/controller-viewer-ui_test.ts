const publicDirectory = new URL(
  "../src/controller-viewer/public/",
  import.meta.url,
);

function source(name: string): string {
  return Deno.readTextFileSync(new URL(name, publicDirectory));
}

Deno.test("viewer uses the pinned Three.js CDN without a local vendor copy", () => {
  const app = source("app.js");
  if (
    !app.includes(
      'from "https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js"',
    )
  ) {
    throw new Error("pinned Three.js CDN import is missing");
  }
  if (app.includes("vendor/three")) {
    throw new Error("viewer still references a local Three.js vendor copy");
  }
});

Deno.test("viewer exposes both controllers and the head selector", () => {
  const html = source("index.html");
  for (const sourceId of [0, 1, 2]) {
    if (!html.includes(`data-source="${sourceId}"`)) {
      throw new Error(`source selector ${sourceId} is missing`);
    }
  }
  if (!html.includes('id="simulation-toggle"')) {
    throw new Error("virtual NOLO start/stop control is missing");
  }
});

Deno.test("viewer DOM references resolve and use explicit pose validity", () => {
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
  if (
    !app.includes("frame.communication_fresh") ||
    app.includes("frame.source_online") ||
    app.includes("frame.controller_online")
  ) {
    throw new Error("viewer source-online API contract is stale");
  }
  if (
    app.includes("frame.pose_usable") || app.includes("optical_tracking_valid")
  ) {
    throw new Error(
      "viewer still consumes removed speculative validity fields",
    );
  }
  if (app.includes("frame.flags")) {
    throw new Error(
      "viewer still consumes the removed compatibility flags",
    );
  }
  if (!app.includes("frame.filtered_position")) {
    throw new Error("viewer does not expose the filtered position diagnostic");
  }
  if (!app.includes("/api/pose-calibration/${sourceId}")) {
    throw new Error("first pose calibration cannot request a missing bias");
  }
  if (!app.includes("FUSION_CALIBRATION_DELAY_MS = 1000")) {
    throw new Error("pose calibration does not ignore the first second");
  }
  if (
    !app.includes("latest?.gyro_calibration_complete === true") ||
    !app.includes('originFusionCalibrationStatus = "accepted"')
  ) {
    throw new Error("saved gyroscope bias does not skip repeated calibration");
  }
  if (!app.includes("cancelCalibrationHold();")) {
    throw new Error("source changes do not cancel stale calibration holds");
  }
  if (
    !app.includes("frame.squeeze_pressed") ||
    !app.includes("latest?.squeeze_pressed") ||
    app.includes("const pressed = !isHead && valid && frame.menu_pressed") ||
    !html.includes("Trigger（接管）") ||
    !html.includes("右侧键（标定）") ||
    !html.includes("Menu（夹爪）")
  ) {
    throw new Error("physical button functions are not aligned in the viewer");
  }
  for (
    const endpoint of ["/api/simulation/start", "/api/simulation/stop"]
  ) {
    if (!app.includes(endpoint)) {
      throw new Error(`viewer does not call ${endpoint}`);
    }
  }
  if (
    html.includes('id="moveit-restart-dialog"') ||
    html.includes("docker restart stararm102-moveit-simulation") ||
    app.includes('$("moveit-restart-dialog").showModal()')
  ) {
    throw new Error(
      "stopping simulation still shows the removed manual restart step",
    );
  }
  if (!app.includes("button.disabled = false")) {
    throw new Error("viewer does not restore the simulation control");
  }
  if (
    !app.includes("resetForAutomaticSimulationCalibration") ||
    !app.includes("先预抬升 10 厘米，再开始循环")
  ) {
    throw new Error("virtual NOLO does not reset and explain auto calibration");
  }
  if (
    !app.includes("restoreViewFromBackend(payload)") ||
    !app.includes("payload.humanReferences") ||
    !app.includes("/api/human-reference/${sourceId}") ||
    !app.includes("latestBackendStatus = payload") ||
    !app.includes("restoreViewFromBackend(latestBackendStatus)") ||
    !app.includes('calibration-state").textContent = "已完成（后端会话）"')
  ) {
    throw new Error("virtual pose cannot be restored from backend state");
  }
  if (app.includes("localStorage") || app.includes("sessionStorage")) {
    throw new Error("human-frame calibration leaked into browser storage");
  }
  if (
    !app.includes("const active = frame.communication_fresh === true") ||
    !app.includes("setSimulationState(active, active)") ||
    !app.includes("&& !simulationRequested")
  ) {
    throw new Error(
      "virtual source transitions can leave the simulator button stale",
    );
  }
  const virtualPhaseBranch = app.match(
    /if \(\s*\(simulationRequested \|\| simulationActive\)[\s\S]*?startsWith\("NOLO 虚拟 USB："\)[\s\S]*?return;\s*\}/,
  )?.[0];
  if (!virtualPhaseBranch || virtualPhaseBranch.includes('$("connection")')) {
    throw new Error(
      "normal virtual phase changes still flash as connection failures",
    );
  }
});
