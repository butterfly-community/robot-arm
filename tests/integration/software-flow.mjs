import assert from "node:assert/strict";
import { mkdir, readFile, writeFile } from "node:fs/promises";

const base = process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765";
const websocketBase = base.replace(/^http/, "ws");
const runId = crypto.randomUUID();
const pickPlaceFixture = JSON.parse(
  await readFile(
    new URL(
      "../../backend/nodes/camera/assets/pick-place-scene-geometry.json",
      import.meta.url,
    ),
    "utf8",
  ),
);

function requestIdForRun(value) {
  return value.startsWith("integration-") ? `${value}-${runId}` : value;
}

async function json(path, init) {
  const response = await fetch(`${base}${path}`, init);
  assert.equal(response.ok, true, `${path}: HTTP ${response.status}`);
  return response.json();
}

async function request(path, body, method = "POST") {
  return json(path, {
    method,
    headers: { "content-type": "application/json" },
    body: JSON.stringify({
      ...body,
      request_id: requestIdForRun(body.request_id),
    }),
  });
}

async function snapshot(namespace) {
  return json(`/api/${namespace}/state`);
}

async function waitFor(read, accept, timeoutMs = 60_000) {
  const deadline = Date.now() + timeoutMs;
  let latestError;
  let latestValue;
  while (Date.now() < deadline) {
    try {
      const value = await read();
      latestValue = value;
      if (accept(value)) return value;
    } catch (error) {
      latestError = error;
    }
    await new Promise((resolve) => setTimeout(resolve, 20));
  }
  if (latestError) throw latestError;
  const directory = new URL(
    `../../temp/software-flow-${runId}/`,
    import.meta.url,
  );
  await mkdir(directory, { recursive: true });
  await writeFile(
    new URL("last-state.json", directory),
    JSON.stringify(latestValue ?? null, null, 2),
  );
  assert.fail(
    `state did not converge; last observed state: ${directory.pathname}last-state.json`,
  );
}

function translationError(actual, expected) {
  return Math.hypot(...actual.map((value, index) => value - expected[index]));
}

function transformPoint(pose, point) {
  const [x, y, z, w] = pose.orientation_xyzw;
  const [px, py, pz] = point;
  const cross = [y * pz - z * py, z * px - x * pz, x * py - y * px];
  const twiceCross = [
    y * cross[2] - z * cross[1],
    z * cross[0] - x * cross[2],
    x * cross[1] - y * cross[0],
  ];
  return point.map(
    (value, index) =>
      value +
      2 * (w * cross[index] + twiceCross[index]) +
      pose.position_m[index],
  );
}

async function observeMotionRequest(requestId, run) {
  requestId = requestIdForRun(requestId);
  const states = new Set();
  const socket = new WebSocket(`${websocketBase}/ws/motion`);
  await new Promise((resolve, reject) => {
    socket.onopen = resolve;
    socket.onerror = reject;
  });
  socket.onmessage = (event) => {
    socket.send("next");
    const snapshot = JSON.parse(event.data);
    const status = snapshot.values?.motion_status;
    if (status?.request_id === requestId) states.add(status.state);
  };
  try {
    const result = await run();
    await waitFor(
      async () => states,
      (observed) =>
        ["planning", "executing", "succeeded"].every((state) =>
          observed.has(state),
        ),
    );
    return result;
  } finally {
    socket.close();
  }
}

async function observeManipulation(requestId, run) {
  requestId = requestIdForRun(requestId);
  const socket = new WebSocket(`${websocketBase}/ws/perception`);
  await new Promise((resolve, reject) => {
    socket.onopen = resolve;
    socket.onerror = reject;
  });
  try {
    const terminal = new Promise((resolve, reject) => {
      socket.onmessage = (event) => {
        socket.send("next");
        const state = JSON.parse(event.data).values?.manipulation_state;
        if (state?.request_id !== requestId) return;
        if (state.state === "succeeded") resolve(state);
        if (state.state === "failed" || state.state === "cancelled") {
          reject(
            new Error(state.original_error ?? `manipulation ${state.state}`),
          );
        }
      };
    });
    const accepted = await run();
    assert.equal(accepted.accepted, true);
    return await terminal;
  } finally {
    socket.close();
  }
}

for (const page of [
  "tracking",
  "spatial",
  "perception",
  "motion",
  "arm-execution",
]) {
  const response = await waitFor(
    () => fetch(`${base}/${page}/`),
    (value) => value.ok,
  );
  assert.match(response.headers.get("content-type") ?? "", /text\/html/);
}

await waitFor(
  () => json("/api/system/readiness"),
  (state) => state.values.system_readiness?.ready === true,
);

await request("/api/arm-execution/disconnect", {
  schema_version: 3,
  request_id: "integration-disconnect",
  action: "disconnect",
  fields: {},
});

const malformedExecution = await fetch(`${base}/api/arm-execution/connect`, {
  method: "POST",
  headers: { "content-type": "application/json" },
  body: JSON.stringify({
    schema_version: 3,
    request_id: "integration-malformed-execution",
    fields: { port: "/dev/ttyUSB0" },
  }),
});
assert.equal(malformedExecution.status, 400);
assert.match(
  (await malformedExecution.json()).original_error,
  /missing field `action`/,
);

const tracking = await waitFor(
  () => snapshot("tracking"),
  (state) => state.namespace === "tracking",
);
const perceptionBefore = await snapshot("perception");
const originalPerception = perceptionBefore.values.perception_state;
const originalCamera = perceptionBefore.values.camera_state;
const originalPerceptionConfigVersion =
  originalPerception?.service?.config_version ?? 0;
let refreshedCameras = await request("/api/perception/camera", {
  schema_version: 3,
  request_id: "integration-camera-refresh",
  action: "refresh",
  source_id: null,
  color_profile_key: null,
  depth_profile_key: null,
});
assert.equal(refreshedCameras.original_error, null);
let simulationCamera = refreshedCameras.value.available_sources.find(
  (source) => source.source_id === "simulation:pick-place-scene",
);
assert.ok(simulationCamera, "camera discovery includes the simulation adapter");
const resetSimulationCamera = await request("/api/perception/camera", {
  schema_version: 3,
  request_id: "integration-camera-reset-before-use",
  action: "reset",
  source_id: simulationCamera.source_id,
  color_profile_key: null,
  depth_profile_key: null,
});
assert.equal(resetSimulationCamera.original_error, null);
refreshedCameras = await request("/api/perception/camera", {
  schema_version: 3,
  request_id: "integration-camera-refresh-after-reset",
  action: "refresh",
  source_id: null,
  color_profile_key: null,
  depth_profile_key: null,
});
assert.equal(refreshedCameras.original_error, null);
simulationCamera = refreshedCameras.value.available_sources.find(
  (source) => source.source_id === "simulation:pick-place-scene",
);
assert.ok(simulationCamera, "reset keeps the simulation adapter available");
const selectedCamera = await request("/api/perception/camera", {
  schema_version: 3,
  request_id: "integration-camera-select",
  action: "select",
  source_id: simulationCamera.source_id,
  color_profile_key: "color:1280x720:rgb8:10",
  depth_profile_key: "depth:1280x720:z16le:10",
  output_frames_per_second: 1,
  driver_parameters: null,
});
assert.equal(selectedCamera.original_error, null);
assert.equal(selectedCamera.value.output_frames_per_second, 1);
assert.deepEqual(
  selectedCamera.value.configurations.find(
    (configuration) => configuration.source_id === simulationCamera.source_id,
  ).driver_parameters,
  [],
);
const startedCamera = await request("/api/perception/camera", {
  schema_version: 3,
  request_id: "integration-camera-connect",
  action: "connect",
  source_id: simulationCamera.source_id,
  color_profile_key: "color:1280x720:rgb8:10",
  depth_profile_key: "depth:1280x720:z16le:10",
});
assert.equal(startedCamera.original_error, null);
const videoFrames = await new Promise((resolve, reject) => {
  const timer = setTimeout(
    () => reject(new Error("camera video frame timeout")),
    2_500,
  );
  const socket = new WebSocket(
    `${base.replace(/^http/, "ws")}/ws/camera-video`,
  );
  socket.binaryType = "arraybuffer";
  let frames = 0;
  socket.onmessage = (event) => {
    if (event.data instanceof ArrayBuffer) frames += 1;
    if (frames >= 5) {
      clearTimeout(timer);
      socket.close();
      resolve(frames);
    }
  };
  socket.onerror = () => {
    clearTimeout(timer);
    reject(new Error("camera video websocket failed"));
  };
});
assert.ok(
  videoFrames >= 5,
  "camera video remains at capture rate while perception output is 1 FPS",
);
const perceptionModelConfigured = await request("/api/perception/request", {
  schema_version: 3,
  request_id: "integration-perception-model-config",
  action: "apply",
  // The fixture owns its matching prompts; these are not production categories.
  classes: [
    pickPlaceFixture.cube.recognition_prompt,
    pickPlaceFixture.bin.placement_region_prompt,
  ],
  placement_labels: [pickPlaceFixture.bin.placement_region_prompt],
  grasp_collision_distance_m:
    pickPlaceFixture.perception.grasp_collision_distance_m,
});
assert.equal(perceptionModelConfigured.original_error, null);
const modelOnlyPerceptionSnapshot = await waitFor(
  () => snapshot("perception"),
  (state) =>
    state.values.perception_state?.service?.config_version >
    originalPerceptionConfigVersion,
);
assert.equal(
  modelOnlyPerceptionSnapshot.values.perception_state.source_id,
  "simulation:pick-place-scene",
);
assert.equal(modelOnlyPerceptionSnapshot.values.perception_state.enabled, true);
await waitFor(
  () => snapshot("perception"),
  (state) =>
    state.values.perception_state?.color_frame != null &&
    state.values.perception_state?.depth_frame != null &&
    state.values.perception_state?.calibrated === true,
);
const obsoleteRun = request("/api/perception/request", {
  schema_version: 3,
  request_id: "integration-obsolete-perception",
  action: "refresh",
});
await waitFor(
  () => snapshot("perception"),
  (state) => state.values.perception_state?.task_state === "executing",
);
const changedDuringRun = await request("/api/perception/request", {
  schema_version: 3,
  request_id: "integration-config-during-inference",
  action: "apply",
  classes: [
    pickPlaceFixture.cube.recognition_prompt,
    pickPlaceFixture.bin.placement_region_prompt,
  ],
  placement_labels: [pickPlaceFixture.bin.placement_region_prompt],
});
assert.equal(changedDuringRun.original_error, null);
assert.match((await obsoleteRun).original_error, /已丢弃旧结果/);
assert.deepEqual(
  (await snapshot("perception")).values.world_scene.objects,
  [],
  "an obsolete asynchronous inference result cannot repopulate the cleared scene",
);
const firstPerceptionRun = await request("/api/perception/request", {
  schema_version: 3,
  request_id: "integration-perception-run",
  action: "refresh",
  classes: null,
  placement_labels: null,
});
assert.equal(firstPerceptionRun.original_error, null);
const readyPerceptionSnapshot = await waitFor(
  () => snapshot("perception"),
  (state) =>
    state.values.perception_state?.enabled === true &&
    state.values.perception_state?.source_id ===
      "simulation:pick-place-scene" &&
    state.values.perception_state?.service?.config_version >
      originalPerceptionConfigVersion &&
    state.values.perception_state?.last_scene_sequence != null &&
    state.values.perception_state.last_scene_sequence ===
      state.values.world_scene?.sequence &&
    state.values.world_scene?.objects?.some(
      (object) =>
        object.label === "red cube" && object.grasp_candidates.length > 0,
    ) &&
    // A prompt can match both the interior and rim; instance count is not fixed.
    state.values.world_scene?.placement_regions?.length > 0,
);
const perceptionScene = readyPerceptionSnapshot.values.world_scene;
const perceptionSnapshot = readyPerceptionSnapshot;
const manualSceneSequence =
  perceptionSnapshot.values.perception_state.last_scene_sequence;
await new Promise((resolve) => setTimeout(resolve, 1_200));
const afterAdditionalFrames = await snapshot("perception");
assert.equal(
  afterAdditionalFrames.values.perception_state.last_scene_sequence,
  manualSceneSequence,
  "new camera frames do not run YOLO or GraspGenX automatically",
);
const simulationColorProfile = simulationCamera.profiles.find(
  (profile) => profile.stream === "color" && profile.is_default,
);
const simulationDepthProfile = simulationCamera.profiles.find(
  (profile) => profile.stream === "depth" && profile.is_default,
);
assert.ok(simulationColorProfile);
assert.ok(simulationDepthProfile);
assert.deepEqual(
  [simulationColorProfile.width, simulationColorProfile.height],
  [1_920, 1_080],
  "simulation camera reports its maximum profile as the default",
);
assert.equal(
  perceptionSnapshot.values.perception_state.color_frame.width,
  1_280,
);
assert.equal(
  perceptionSnapshot.values.perception_state.depth_frame.height,
  720,
);
assert.equal(
  typeof perceptionSnapshot.values.perception_state.point_count,
  "number",
);
for (const asset of ["color.png", "overlay.png", "depth.png"]) {
  const response = await fetch(`${base}/api/perception/assets/${asset}`);
  assert.equal(response.ok, true, `perception asset ${asset}`);
  assert.match(response.headers.get("content-type") ?? "", /^image\/png/);
  const bytes = new Uint8Array(await response.arrayBuffer());
  assert.deepEqual(
    Array.from(bytes.slice(0, 8)),
    [137, 80, 78, 71, 13, 10, 26, 10],
  );
}
assert.equal(perceptionSnapshot.values.perception_state.depth_scale_m, 0.001);
const calibrationModel = readyPerceptionSnapshot.values.robot_model_info;
assert.ok(calibrationModel.calibration_targets.length > 0);
const simulationOracle = JSON.parse(
  await readFile(
    new URL(
      "../../backend/nodes/camera/assets/simulation-cameras.json",
      import.meta.url,
    ),
    "utf8",
  ),
).find((source) => source.source_id === "simulation:pick-place-scene");
assert.ok(simulationOracle);
const calibrationBoard = {
  pattern: "charuco",
  dictionary: "DICT_4X4_50",
  squares_x: 5,
  squares_y: 5,
  square_size_m: 0.015,
  marker_size_m: 0.011,
  measured_width_m: 0.075,
  measured_height_m: 0.075,
};

async function useSimulationProfile(width, height) {
  const colorProfileKey = `color:${width}x${height}:rgb8:10`;
  const depthProfileKey = `depth:${width}x${height}:z16le:10`;
  const selected = await request("/api/perception/camera", {
    schema_version: 3,
    request_id: `integration-camera-profile-${width}x${height}`,
    action: "select",
    source_id: "simulation:pick-place-scene",
    color_profile_key: colorProfileKey,
    depth_profile_key: depthProfileKey,
    output_frames_per_second: 1,
    driver_parameters: null,
  });
  assert.equal(selected.original_error, null);
  const connected = await request("/api/perception/camera", {
    schema_version: 3,
    request_id: `integration-camera-reconnect-${width}x${height}`,
    action: "connect",
    source_id: "simulation:pick-place-scene",
    color_profile_key: colorProfileKey,
    depth_profile_key: depthProfileKey,
  });
  assert.equal(connected.original_error, null);
  await waitFor(
    () => snapshot("perception"),
    (state) =>
      state.values.perception_state?.color_frame?.width === width &&
      state.values.perception_state?.color_frame?.height === height &&
      state.values.perception_state?.depth_frame?.width === width &&
      state.values.perception_state?.depth_frame?.height === height,
  );
}

await useSimulationProfile(1_920, 1_080);

async function runAutomaticCalibration(run) {
  const started = await request("/api/perception/calibration", {
    schema_version: 3,
    request_id: `integration-automatic-calibration-${run}`,
    action: "start",
    board: calibrationBoard,
    camera_source_id: "simulation:pick-place-scene",
    robot_model_revision: calibrationModel.model_revision,
    calibration_tool_id: "integration-fixture",
  });
  assert.equal(started.original_error, null);
  const completed = await waitFor(
    () => snapshot("perception"),
    (state) =>
      ["awaiting_confirmation", "failed"].includes(
        state.values.calibration_state?.phase,
      ),
    300_000,
  );
  const calibration = completed.values.calibration_state;
  const evidenceDirectory = new URL(
    `../../temp/software-flow-${runId}/`,
    import.meta.url,
  );
  await mkdir(evidenceDirectory, { recursive: true });
  await writeFile(
    new URL(`calibration-${run}.json`, evidenceDirectory),
    JSON.stringify(calibration, null, 2),
  );
  assert.notEqual(
    calibration.phase,
    "failed",
    calibration.original_error ?? calibration.stage_message,
  );
  assert.equal(
    calibration.target_count,
    calibrationModel.calibration_targets.length,
  );
  assert.equal(calibration.observations.length, calibration.target_count);
  assert.equal(
    calibration.solved_result.sample_count,
    calibration.target_count,
  );
  assert.equal(
    calibration.solved_result.translation_residuals_m.length,
    calibration.target_count,
  );
  assert.equal(
    calibration.solved_result.rotation_residuals_rad.length,
    calibration.target_count,
  );
  const frameBeforeApply = completed.values.perception_state.last_frame_time_ns;
  const applied = await request("/api/perception/calibration", {
    schema_version: 3,
    request_id: `integration-apply-calibration-${run}`,
    action: "apply",
    board: null,
    camera_source_id: null,
    robot_model_revision: null,
    calibration_tool_id: null,
  });
  assert.equal(applied.original_error, null);
  await waitFor(
    () => snapshot("perception"),
    (state) =>
      state.values.calibration_state?.phase === "applied" &&
      state.values.perception_state?.last_frame_time_ns > frameBeforeApply,
  );
  return calibration.solved_result;
}

const firstCalibration = await runAutomaticCalibration("first");
const secondCalibration = await runAutomaticCalibration("second");
await useSimulationProfile(1_280, 720);
const thirdCalibration = await runAutomaticCalibration("1280");
for (const calibration of [
  firstCalibration,
  secondCalibration,
  thirdCalibration,
]) {
  // User-required 0.1 mm synthetic calibration accuracy, not a motion gate.
  assert.ok(
    translationError(
      calibration.camera_in_base.position_m,
      simulationOracle.camera_in_base.position_m,
    ) <= 0.0001,
    "solved simulated camera translation is within 0.1 mm of the oracle",
  );
  const { camera_matrix: k, width, height } = simulationOracle.calibration;
  for (const u of [0, width - 1])
    for (const v of [0, height - 1]) {
      const ray = [(u - k[2]) / k[0], (v - k[5]) / k[4], 1];
      const direction = transformPoint(
        { ...simulationOracle.camera_in_base, position_m: [0, 0, 0] },
        ray,
      );
      const depth =
        -simulationOracle.camera_in_base.position_m[2] / direction[2];
      assert.ok(
        depth > 0,
        "fixture ground intersects the visible camera frustum",
      );
      const point = ray.map((value) => value * depth);
      assert.ok(
        translationError(
          transformPoint(calibration.camera_in_base, point),
          transformPoint(simulationOracle.camera_in_base, point),
        ) <= 0.0001,
        "combined camera translation and rotation displace visible ground by at most 0.1 mm",
      );
    }
  assert.ok(
    translationError(
      calibration.board_in_calibration_tool.position_m,
      simulationOracle.board_in_tool.position_m,
    ) <= 0.0001,
    "solved ChArUco origin is within 0.1 mm of the simulation oracle",
  );
  for (const x of [0, calibrationBoard.measured_width_m])
    for (const y of [0, calibrationBoard.measured_height_m]) {
      assert.ok(
        translationError(
          transformPoint(calibration.board_in_calibration_tool, [x, y, 0]),
          transformPoint(simulationOracle.board_in_tool, [x, y, 0]),
        ) <= 0.0001,
        "combined board translation and rotation displace each board corner by at most 0.1 mm",
      );
    }
}
await useSimulationProfile(1_280, 720);
const sceneSequenceBeforeCalibrationRefresh = (await snapshot("perception"))
  .values.perception_state.last_scene_sequence;
const calibratedPerceptionRun = await request("/api/perception/request", {
  schema_version: 3,
  request_id: "integration-perception-run-after-calibration",
  action: "refresh",
  classes: null,
  placement_labels: null,
});
assert.equal(calibratedPerceptionRun.original_error, null);
const calibratedPerception = await waitFor(
  () => snapshot("perception"),
  (state) =>
    state.values.perception_state?.last_scene_sequence >
      sceneSequenceBeforeCalibrationRefresh &&
    state.values.world_scene?.objects?.length > 0,
);
const calibratedScene = calibratedPerception.values.world_scene;
const graspable = calibratedScene.objects.find(
  (object) => object.grasp_candidates.length > 0,
);
const placementRegion = calibratedScene.placement_regions[0];
assert.ok(graspable, "simulation perception scene has a graspable object");
assert.ok(
  placementRegion,
  "simulation perception scene has a placement region",
);
assert.ok(
  graspable.size_m.every((value) => Number.isFinite(value) && value > 0),
  "the observed model cloud has a nondegenerate extent; visible surfaces are not the complete solid",
);
await request("/api/motion/mode", {
  schema_version: 3,
  request_id: "integration-perception-mode",
  mode: "perception",
});
const pickPlaceResult = await observeManipulation(
  "integration-pick-place",
  () =>
    request("/api/perception/pick-place", {
      schema_version: 3,
      request_id: "integration-pick-place",
      object_id: graspable.object_id,
      scene_sequence: calibratedScene.sequence,
      placement_region_id: placementRegion.region_id,
    }),
);
assert.equal(pickPlaceResult.state, "succeeded");
assert.equal(pickPlaceResult.stage, "pick and place complete");
assert.ok(pickPlaceResult.solution_count > 0);
assert.equal(typeof pickPlaceResult.selected_cost, "number");
assert.equal(pickPlaceResult.object_id, graspable.object_id);
assert.equal(pickPlaceResult.placement_region_id, placementRegion.region_id);
assert.deepEqual(pickPlaceResult.pick_position_m, [
  graspable.pose.position_m[0],
  graspable.pose.position_m[1],
  graspable.pose.position_m[2],
]);
const expectedPlacePosition = [
  placementRegion.pose.position_m[0],
  placementRegion.pose.position_m[1],
  Math.max(placementRegion.pose.position_m[2], 0.1),
];
assert.deepEqual(
  pickPlaceResult.place_position_m.slice(0, 2),
  expectedPlacePosition.slice(0, 2),
);
assert.ok(
  Math.abs(pickPlaceResult.place_position_m[2] - expectedPlacePosition[2]) <=
    Number.EPSILON,
);

// This is an API/execution contract test, not proof of gripping. The independent
// FK/mesh oracle in tools/graspgenx/verify-executed-pick-place.py checks recorded
// joint feedback, opposing contact, lift and release; orientation is not fixed.
const executionPickPlace = await waitFor(
  () => snapshot("arm-execution"),
  (state) => {
    const model = state.values.robot_model_info;
    const target = model.named_targets.find((item) => item.key === "work");
    return (
      target != null &&
      model.joints.every(
        (joint, index) =>
          Math.abs(
            state.values.arm_state.joints_rad[index] -
              target.joint_positions_rad[joint.key],
          ) <=
          Math.PI / 180,
      )
    );
  },
);
const executionModel = executionPickPlace.values.robot_model_info;
const pickPlaceWorkTarget = executionModel.named_targets.find(
  (target) => target.key === "work",
);
assert.ok(pickPlaceWorkTarget, "robot model declares the work target");
assert.ok(
  executionModel.joints.every(
    (joint, index) =>
      Math.abs(
        executionPickPlace.values.arm_state.joints_rad[index] -
          pickPlaceWorkTarget.joint_positions_rad[joint.key],
      ) <=
      Math.PI / 180,
  ),
  "pick-place returns to the declared work target",
);
assert.ok(
  executionModel.tool_actuators.every(
    (actuator, index) =>
      Math.abs(
        executionPickPlace.values.arm_state.actuators_rad[index] -
          pickPlaceWorkTarget.actuator_positions_rad[actuator.key],
      ) < 1e-9,
  ),
  "pick-place returns tool actuators to the declared work target",
);
assert.equal(
  executionPickPlace.values.manipulation_state.object_id,
  graspable.object_id,
);
assert.equal(
  executionPickPlace.values.manipulation_state.placement_region_id,
  placementRegion.region_id,
);
assert.deepEqual(
  executionPickPlace.values.manipulation_state.pick_position_m,
  pickPlaceResult.pick_position_m,
);
assert.deepEqual(
  executionPickPlace.values.manipulation_state.place_position_m,
  pickPlaceResult.place_position_m,
);
const resetPerceptionCamera = await request("/api/perception/camera", {
  schema_version: 3,
  request_id: "integration-reset-simulation-calibration",
  action: "reset",
  source_id: "simulation:pick-place-scene",
  classes: null,
  placement_labels: null,
});
assert.equal(resetPerceptionCamera.original_error, null);
const selectedResetCamera = await request("/api/perception/camera", {
  schema_version: 3,
  request_id: "integration-select-reset-simulation-camera",
  action: "select",
  source_id: "simulation:pick-place-scene",
  color_profile_key: null,
  depth_profile_key: null,
  output_frames_per_second: 1,
  driver_parameters: null,
});
assert.equal(selectedResetCamera.original_error, null);
const startedResetCamera = await request("/api/perception/camera", {
  schema_version: 3,
  request_id: "integration-start-reset-simulation-camera",
  action: "connect",
  source_id: "simulation:pick-place-scene",
  color_profile_key: null,
  depth_profile_key: null,
});
assert.equal(startedResetCamera.original_error, null);
await waitFor(
  () => snapshot("perception"),
  (state) =>
    state.values.calibration_state?.phase === "idle" &&
    state.values.perception_state?.calibrated === true,
);
await request("/api/perception/request", {
  schema_version: 3,
  request_id: "integration-perception-stop",
  action: "disconnect",
  classes: null,
  placement_labels: null,
});
await waitFor(
  () => snapshot("perception"),
  (state) => state.values.perception_state?.enabled === false,
);
await request("/api/perception/camera", {
  schema_version: 3,
  request_id: "integration-camera-unselect",
  action: "unselect",
  source_id: null,
  color_profile_key: null,
  depth_profile_key: null,
});
if (originalCamera?.selected_source_id) {
  await request("/api/perception/camera", {
    schema_version: 3,
    request_id: "integration-camera-restore-refresh",
    action: "refresh",
    source_id: null,
    color_profile_key: null,
    depth_profile_key: null,
  });
  await request("/api/perception/camera", {
    schema_version: 3,
    request_id: "integration-camera-restore-select",
    action: "select",
    source_id: originalCamera.selected_source_id,
    color_profile_key: originalCamera.selected_color_profile_key,
    depth_profile_key: originalCamera.selected_depth_profile_key,
    output_frames_per_second: originalCamera.output_frames_per_second ?? null,
    driver_parameters:
      originalCamera.configurations?.find(
        (configuration) =>
          configuration.source_id === originalCamera.selected_source_id,
      )?.driver_parameters ?? null,
  });
  if (originalCamera.streaming)
    await request("/api/perception/camera", {
      schema_version: 3,
      request_id: "integration-camera-restore-connect",
      action: "connect",
      source_id: originalCamera.selected_source_id,
      color_profile_key: null,
      depth_profile_key: null,
    });
}
if (originalPerception?.enabled) {
  await request("/api/perception/request", {
    schema_version: 3,
    request_id: "integration-perception-restore",
    action: "apply",
    classes: originalPerception.classes,
    placement_labels: originalPerception.placement_labels,
    grasp_collision_distance_m: originalPerception.grasp_collision_distance_m,
  });
}
const discoveredSources = tracking.values.discovery_state?.sources ?? [];
for (const [component, capability] of [
  ["position", "position_capable"],
  ["orientation", "orientation_capable"],
]) {
  const unsupported = discoveredSources.find((source) => !source[capability]);
  if (!unsupported) continue;
  const rejected = await request("/api/tracking/pose-source", {
    schema_version: 3,
    request_id: `integration-reject-${component}-source`,
    action: "select",
    component,
    driver_id: unsupported.driver_id,
    device_id: unsupported.device_id,
    source_id: unsupported.source_id,
  });
  assert.match(rejected.original_error, /不提供对应位姿能力/);
}
const selectedInput = tracking.values.discovery_state?.position_source;
if (selectedInput) {
  await request("/api/tracking/pose-source", {
    schema_version: 3,
    request_id: "integration-input-unselect",
    action: "unselect",
    component: "position",
    driver_id: selectedInput.driver_id,
    device_id: selectedInput.device_id,
    source_id: selectedInput.source_id,
  });
  await waitFor(
    () => snapshot("spatial"),
    (state) => state.values.transformed_control?.active === false,
  );
  await request("/api/tracking/pose-source", {
    schema_version: 3,
    request_id: "integration-input-restore",
    action: "select",
    component: "position",
    driver_id: selectedInput.driver_id,
    device_id: selectedInput.device_id,
    source_id: selectedInput.source_id,
  });
  await waitFor(
    () => snapshot("tracking"),
    (state) =>
      state.values.discovery_state?.position_source_id ===
      selectedInput.source_id,
  );
}

await waitFor(
  () => snapshot("motion"),
  (state) =>
    state.values.motion_state?.current_tool_pose != null &&
    state.values.motion_state?.service?.has_output === true,
);

await new Promise((resolve, reject) => {
  const socket = new WebSocket(`${websocketBase}/ws/motion`);
  const timer = setTimeout(
    () => reject(new Error("motion WebSocket did not send a snapshot")),
    10_000,
  );
  socket.onmessage = (event) => {
    const message = JSON.parse(event.data);
    assert.equal(message.namespace, "motion");
    clearTimeout(timer);
    socket.close();
    resolve();
  };
  socket.onerror = reject;
});

const initialSpatial = await snapshot("spatial");
const originalScale =
  initialSpatial.values.spatial_config_state.translation_scale;
const changed = await request(
  "/api/spatial/config",
  {
    schema_version: 3,
    request_id: "integration-scale",
    patch: { translation_scale: 0.25 },
  },
  "PATCH",
);
assert.equal(changed.value.translation_scale, 0.25);
await request(
  "/api/spatial/config",
  {
    schema_version: 3,
    request_id: "integration-scale-restore",
    patch: { translation_scale: originalScale },
  },
  "PATCH",
);

await request("/api/motion/mode", {
  schema_version: 3,
  request_id: "integration-manual",
  mode: "manual",
});

const beforeBindings = await snapshot("tracking");
const selectedRuntimeSource =
  beforeBindings.values.discovery_state?.sources?.find((source) =>
    source.available_components?.some(
      (component) => component.action_type === "boolean",
    ),
  );
const booleanComponents =
  selectedRuntimeSource?.available_components?.filter(
    (component) => component.action_type === "boolean",
  ) ?? [];
const originalBindings = (beforeBindings.values.discovery_state?.bindings ?? [])
  .filter((binding) => binding.configured_components.length > 0)
  .map((binding) => ({
    action: binding.action,
    action_type: binding.action_type,
    source_id: binding.source_id,
    component_paths: binding.configured_components,
    invert: binding.invert,
  }));
const originalFeedbackBindings =
  beforeBindings.values.discovery_state?.feedback_bindings?.map((binding) => ({
    action: binding.action,
    source_id: binding.source_id,
    capability_path: binding.capability_path,
  })) ?? [];
const applyBindings = (
  requestId,
  bindings,
  feedbackBindings = originalFeedbackBindings,
) =>
  request("/api/tracking/bindings", {
    schema_version: 3,
    request_id: requestId,
    bindings,
    feedback_bindings: feedbackBindings,
  });
if (selectedRuntimeSource?.source_id && booleanComponents.length > 0) {
  try {
    const booleanResult = await applyBindings("integration-binding-boolean", [
      {
        action: "start_stop",
        action_type: "boolean",
        source_id: selectedRuntimeSource.source_id,
        component_paths: [booleanComponents[0].path],
        invert: false,
      },
    ]);
    assert.equal(booleanResult.original_error, null);
    await waitFor(
      () => snapshot("tracking"),
      (state) => {
        const binding = state.values.discovery_state?.bindings?.find(
          (value) => value.action === "start_stop",
        );
        return (
          binding?.active === true &&
          binding.configured_components.includes(booleanComponents[0].path)
        );
      },
    );
    if (booleanComponents.length > 1) {
      const pairResult = await applyBindings("integration-binding-pair", [
        {
          action: "move_left_right",
          action_type: "float",
          source_id: selectedRuntimeSource.source_id,
          component_paths: [
            booleanComponents[0].path,
            booleanComponents[1].path,
          ],
          invert: true,
        },
      ]);
      assert.equal(pairResult.original_error, null);
      const pairBinding = pairResult.value.find(
        (value) => value.action === "move_left_right",
      );
      assert.equal(pairBinding.active, true);
      assert.equal(
        booleanComponents
          .slice(0, 2)
          .every((component) =>
            pairBinding.configured_components.includes(component.path),
          ),
        true,
      );
    }
  } finally {
    const restored = await applyBindings(
      "integration-bindings-restore",
      originalBindings,
    );
    assert.equal(restored.original_error, null);
  }
}

try {
  const virtualResult = await applyBindings(
    "integration-virtual-feedback",
    originalBindings,
    [
      {
        action: "primary_tool",
        source_id: "virtual-feedback",
        capability_path: "feedback/virtual",
      },
    ],
  );
  assert.equal(virtualResult.original_error, null);
  // Software joint feedback is not measured force. Use the existing explicit
  // feedback demo rather than expecting a snapshot to invent hardware telemetry.
  const feedbackDemo = await request("/api/tracking/simulation", {
    schema_version: 3,
    request_id: "integration-virtual-feedback-demo",
    enabled: true,
    item: "primary_tool_feedback",
  });
  assert.equal(feedbackDemo.original_error, null);
  await waitFor(
    () => snapshot("tracking"),
    (state) =>
      state.values.discovery_state?.feedback_bindings?.some(
        (binding) =>
          binding.source_id === "virtual-feedback" &&
          binding.capability_path === "feedback/virtual" &&
          binding.applicable === true,
      ) &&
      state.values.discovery_state?.virtual_feedback?.action === "primary_tool",
  );
} finally {
  await request("/api/tracking/simulation", {
    schema_version: 3,
    request_id: "integration-virtual-feedback-demo-stop",
    enabled: false,
    item: null,
  });
  const restored = await applyBindings(
    "integration-virtual-feedback-restore",
    originalBindings,
    originalFeedbackBindings,
  );
  assert.equal(restored.original_error, null);
}

const motion = await waitFor(
  () => snapshot("motion"),
  (state) =>
    state.values.robot_model_info &&
    state.values.arm_state &&
    state.values.motion_state?.service?.has_input === true &&
    state.values.motion_state?.service?.has_output === true,
);
const model = motion.values.robot_model_info;
const before = motion.values.arm_state;
await request("/api/motion/mode", {
  schema_version: 3,
  request_id: "integration-manual-mode",
  mode: "manual",
});
assert.equal(model.joints.length, before.joints_rad.length);
const moved = [...before.joints_rad];
moved[0] += Math.PI / 36;
const motionResult = await observeMotionRequest(
  "integration-joint-five-degrees",
  () =>
    request("/api/motion/request", {
      schema_version: 3,
      request_id: "integration-joint-five-degrees",
      model_revision: model.model_revision,
      joints: model.joints.map((joint, index) => ({
        joint_key: joint.key,
        position_rad: moved[index],
      })),
      actuators: model.tool_actuators.map((item, index) => ({
        actuator_key: item.key,
        position_rad: before.actuators_rad[index],
      })),
      options: {},
      action: "apply",
    }),
);
assert.equal(motionResult.value.state, "succeeded");
await waitFor(
  () => snapshot("arm-execution"),
  (state) =>
    Math.abs(state.values.arm_state.joints_rad[0] - moved[0]) <= Math.PI / 180,
);

const queuedAway = request("/api/motion/request", {
  schema_version: 3,
  request_id: "integration-fifo-away",
  model_revision: model.model_revision,
  joints: model.joints.map((joint, index) => ({
    joint_key: joint.key,
    position_rad: before.joints_rad[index],
  })),
  actuators: model.tool_actuators.map((item, index) => ({
    actuator_key: item.key,
    position_rad: before.actuators_rad[index],
  })),
  options: {},
  action: "apply",
});
await waitFor(
  () => snapshot("motion"),
  (state) =>
    state.values.motion_state.latest_motion.request_id ===
      requestIdForRun("integration-fifo-away") &&
    ["planning", "executing"].includes(
      state.values.motion_state.latest_motion.state,
    ),
);
const queuedReturn = request("/api/motion/request", {
  schema_version: 3,
  request_id: "integration-fifo-return",
  model_revision: model.model_revision,
  joints: model.joints.map((joint, index) => ({
    joint_key: joint.key,
    position_rad: moved[index],
  })),
  actuators: model.tool_actuators.map((item, index) => ({
    actuator_key: item.key,
    position_rad: before.actuators_rad[index],
  })),
  options: {},
  action: "apply",
});
const [queuedAwayResult, queuedReturnResult] = await Promise.all([
  queuedAway,
  queuedReturn,
]);
assert.equal(queuedAwayResult.value.state, "succeeded");
assert.equal(queuedReturnResult.value.state, "succeeded");
await waitFor(
  () => snapshot("arm-execution"),
  (state) =>
    Math.abs(state.values.arm_state.joints_rad[0] - moved[0]) <= Math.PI / 180,
);

const actuator = model.tool_actuators[0];
assert.equal(model.tcp_frame, "tcp_link");
assert.equal(model.gripper_asset_id, "stararm-102-fl");
assert.ok(actuator, "fixture model publishes an actuator");
const actuatorTarget = before.actuators_rad[0] + Math.PI / 36;
const actuatorResult = await request("/api/motion/actuator", {
  schema_version: 3,
  request_id: "integration-actuator-five-degrees",
  model_revision: model.model_revision,
  actuator_key: actuator.key,
  position_rad: actuatorTarget,
});
assert.equal(actuatorResult.original_error, null);
assert.equal(actuatorResult.value.state, "succeeded");
const atActuatorTarget = await waitFor(
  () => snapshot("arm-execution"),
  (state) => state.values.arm_state.actuators_rad[0] > before.actuators_rad[0],
);

const defaultTarget = model.named_targets.find(
  (target) => target.key === "default",
);
assert.ok(defaultTarget, "model publishes the default target");
assert.equal(defaultTarget.joint_positions_rad.joint3, 0);
assert.equal(
  Object.keys(defaultTarget.joint_positions_rad).length,
  model.joints.length,
);
assert.equal(defaultTarget.actuator_positions_rad.gripper, 0);
const defaultResult = await request("/api/motion/request", {
  schema_version: 3,
  request_id: "integration-default",
  model_revision: model.model_revision,
  joints: model.joints.map((joint) => ({
    joint_key: joint.key,
    position_rad: defaultTarget.joint_positions_rad[joint.key],
  })),
  actuators: model.tool_actuators.map((item) => ({
    actuator_key: item.key,
    position_rad: defaultTarget.actuator_positions_rad[item.key],
  })),
  options: {},
  action: "apply",
});
assert.equal(defaultResult.value.state, "succeeded");
const atDefault = await waitFor(
  () => snapshot("arm-execution"),
  (state) =>
    model.joints.every(
      (joint, index) =>
        Math.abs(
          state.values.arm_state.joints_rad[index] -
            defaultTarget.joint_positions_rad[joint.key],
        ) <=
        Math.PI / 180,
    ) &&
    Math.abs(state.values.arm_state.actuators_rad[0]) <= (2 * Math.PI) / 180,
);

const cancellable = [...atDefault.values.arm_state.joints_rad];
cancellable[0] += Math.PI / 18;
const interruptedRequest = request("/api/motion/request", {
  schema_version: 3,
  request_id: "integration-interrupted-motion",
  model_revision: model.model_revision,
  joints: model.joints.map((joint, index) => ({
    joint_key: joint.key,
    position_rad: cancellable[index],
  })),
  actuators: model.tool_actuators.map((item, index) => ({
    actuator_key: item.key,
    position_rad: atDefault.values.arm_state.actuators_rad[index],
  })),
  options: {},
  action: "apply",
});
await waitFor(
  () => snapshot("motion"),
  (state) =>
    state.values.motion_state.latest_motion.request_id ===
      requestIdForRun("integration-interrupted-motion") &&
    ["planning", "executing"].includes(
      state.values.motion_state.latest_motion.state,
    ),
);
const cancelResult = await request("/api/motion/cancel", {
  schema_version: 3,
  request_id: "integration-cancel",
  action: "cancel",
});
assert.equal(cancelResult.request_id, requestIdForRun("integration-cancel"));
assert.equal(
  cancelResult.value.state,
  "succeeded",
  "cancel request acknowledged; original motion owns its terminal state",
);
const interruptedResult = await interruptedRequest;
assert.equal(
  interruptedResult.request_id,
  requestIdForRun("integration-interrupted-motion"),
);
assert.equal(interruptedResult.value.state, "cancelled");

await request("/api/motion/actuator", {
  schema_version: 3,
  request_id: "integration-actuator-restore",
  model_revision: model.model_revision,
  actuator_key: actuator.key,
  position_rad: before.actuators_rad[0],
});
await waitFor(
  () => snapshot("arm-execution"),
  (state) =>
    Math.abs(
      state.values.arm_state.actuators_rad[0] - before.actuators_rad[0],
    ) <
    Math.abs(
      atActuatorTarget.values.arm_state.actuators_rad[0] -
        before.actuators_rad[0],
    ),
);

const asset = await fetch(
  `${base}/api/motion/assets/${model.visualization.root_path}`,
);
assert.equal(asset.ok, true);
assert.ok((await asset.arrayBuffer()).byteLength > 0);

const simulationItems = [
  ["move_forward_back", true],
  ["move_left_right", true],
  ["move_up_down", true],
  ["tool_pitch", true],
  ["tool_yaw", true],
  ["tool_roll", true],
  ["front_pitch", true],
  ["horizontal_arc", true],
  ["primary_tool_open", true],
  ["primary_tool", true],
  ["start_stop", true],
  ["emergency_stop", true],
  ["primary_tool_feedback", false],
  ["tool_axis_translation", true],
  ["tool_helical_motion", true],
];

for (const [item, prepare] of simulationItems) {
  if (prepare) {
    const prepared = await request("/api/motion/prepare-relative", {
      schema_version: 3,
      request_id: `integration-prepare-${item}`,
    });
    assert.equal(prepared.value.state, "succeeded");
  }
  const started = await request("/api/tracking/simulation", {
    schema_version: 3,
    request_id: `integration-simulation-${item}`,
    enabled: true,
    item,
  });
  assert.equal(started.value.active, true);
  assert.equal(started.value.item, item);
  await waitFor(
    () => snapshot("tracking"),
    (state) =>
      state.values.discovery_state?.simulation?.active === true &&
      state.values.discovery_state.simulation.item === item &&
      state.values.discovery_state.sources.some(
        (source) => source.source_id === "simulation:generic-action-source",
      ),
  );
  if (item !== "primary_tool_feedback") {
    await waitFor(
      () => snapshot("tracking"),
      (state) =>
        state.values.discovery_state?.simulation?.active === true &&
        state.values.discovery_state.simulation.item === item &&
        state.values.discovery_state.simulation.phase === "lifting",
    );
  }
  await waitFor(
    () => snapshot("tracking"),
    (state) => state.values.discovery_state?.simulation?.active === false,
  );
  const restoredTracking = await snapshot("tracking");
  assert.deepEqual(
    restoredTracking.values.discovery_state.bindings
      .filter((binding) => binding.configured_components.length > 0)
      .map((binding) => ({
        action: binding.action,
        action_type: binding.action_type,
        source_id: binding.source_id,
        component_paths: binding.configured_components,
        invert: binding.invert,
      })),
    originalBindings,
  );
  assert.equal(
    restoredTracking.values.discovery_state.sources.some(
      (source) => source.source_id === "simulation:generic-action-source",
    ),
    false,
  );
}

const simulatedExecution = await snapshot("arm-execution");
const simulatedCommand = simulatedExecution.values.transport_state.last_command;
assert.equal(simulatedCommand.model_revision, "stararm-102-fl-v2");
assert.equal(simulatedCommand.joints_rad.length, 6);
assert.equal(simulatedCommand.actuators_rad.length, 1);

const execution = await snapshot("arm-execution");
assert.equal(execution.values.transport_state.connected, false);
assert.equal(execution.values.arm_state.feedback_source, "software");
assert.equal(execution.values.execution_service_state.has_input, true);
console.log("software-flow: ok");
