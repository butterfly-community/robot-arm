import assert from "node:assert/strict";

const base = process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765";
const websocketBase = base.replace(/^http/, "ws");

async function json(path, init) {
  const response = await fetch(`${base}${path}`, init);
  assert.equal(response.ok, true, `${path}: HTTP ${response.status}`);
  return response.json();
}

async function request(path, body, method = "POST") {
  return json(path, {
    method,
    headers: { "content-type": "application/json" },
    body: JSON.stringify(body),
  });
}

async function snapshot(namespace) {
  return json(`/api/${namespace}/state`);
}

async function waitFor(read, accept) {
  const deadline = Date.now() + 60_000;
  let latestError;
  while (Date.now() < deadline) {
    try {
      const value = await read();
      if (accept(value)) return value;
    } catch (error) {
      latestError = error;
    }
    await new Promise((resolve) => setTimeout(resolve, 20));
  }
  if (latestError) throw latestError;
  assert.fail("state did not converge during integration test");
}

async function observeMotionRequest(requestId, run) {
  const states = new Set();
  const socket = new WebSocket(`${websocketBase}/ws/motion`);
  await new Promise((resolve, reject) => {
    socket.onopen = resolve;
    socket.onerror = reject;
  });
  socket.onmessage = (event) => {
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
  const socket = new WebSocket(`${websocketBase}/ws/motion`);
  await new Promise((resolve, reject) => {
    socket.onopen = resolve;
    socket.onerror = reject;
  });
  try {
    const terminal = new Promise((resolve, reject) => {
      socket.onmessage = (event) => {
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

for (const page of ["tracking", "spatial", "motion", "arm-execution"]) {
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
const motionBeforePerception = await snapshot("motion");
const originalPerception = motionBeforePerception.values.perception_state;
const perceptionEnabled = await request("/api/motion/perception", {
  schema_version: 3,
  request_id: "integration-perception-enable",
  action: "apply",
  source_kind: "generated_test_scene",
  source_id: "generated:pick-place-scene",
  classes: null,
});
assert.equal(perceptionEnabled.original_error, null);
await waitFor(
  () => snapshot("motion"),
  (state) =>
    state.values.perception_state?.enabled === true &&
    state.values.perception_state?.source_kind === "generated_test_scene" &&
    state.values.world_scene?.objects?.some(
      (object) => object.label === "red cube" && object.graspable,
    ) &&
    state.values.world_scene?.placement_regions?.length === 1,
);
const perceptionScene = (await snapshot("motion")).values.world_scene;
const graspable = perceptionScene.objects.find((object) => object.graspable);
const placementRegion = perceptionScene.placement_regions[0];
assert.ok(graspable, "generated perception scene has a graspable object");
assert.ok(placementRegion, "generated perception scene has a placement region");
const pickPlaceResult = await observeManipulation(
  "integration-pick-place",
  () =>
    request("/api/motion/pick-place", {
      schema_version: 3,
      request_id: "integration-pick-place",
      object_id: graspable.object_id,
      placement_region_id: placementRegion.region_id,
    }),
);
assert.equal(pickPlaceResult.state, "succeeded");
assert.equal(pickPlaceResult.step, "complete");
assert.equal(pickPlaceResult.object_id, graspable.object_id);
assert.equal(pickPlaceResult.placement_region_id, placementRegion.region_id);
assert.deepEqual(pickPlaceResult.pick_position_m, [
  graspable.pose.position_m[0],
  graspable.pose.position_m[1],
  graspable.pose.position_m[2] + graspable.size_m[2] / 2,
]);
assert.deepEqual(pickPlaceResult.place_position_m, [
  placementRegion.pose.position_m[0],
  placementRegion.pose.position_m[1],
  placementRegion.pose.position_m[2] +
    placementRegion.size_m[2] +
    graspable.size_m[2] / 2,
]);
const executionPickPlace = await snapshot("arm-execution");
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
await request("/api/motion/perception", {
  schema_version: 3,
  request_id: "integration-perception-stop",
  action: "disconnect",
  source_kind: null,
  source_id: null,
  classes: null,
});
await waitFor(
  () => snapshot("motion"),
  (state) => state.values.perception_state?.enabled === false,
);
if (originalPerception?.enabled) {
  await request("/api/motion/perception", {
    schema_version: 3,
    request_id: "integration-perception-restore",
    action: "apply",
    source_kind: originalPerception.source_kind,
    source_id: originalPerception.source_id,
    classes: originalPerception.classes,
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
  const feedbackSnapshot = await fetch(`${base}/api/arm-execution/snapshot`, {
    method: "POST",
  });
  assert.equal(feedbackSnapshot.status, 202);
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
  (state) => Math.abs(state.values.arm_state.joints_rad[0] - moved[0]) < 1e-9,
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
      "integration-fifo-away" &&
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
  (state) => Math.abs(state.values.arm_state.joints_rad[0] - moved[0]) < 1e-9,
);

const actuator = model.tool_actuators[0];
assert.equal(model.tcp_frame, "tcp_link");
assert.ok(actuator, "fixture model publishes an actuator");
const actuatorTarget = before.actuators_rad[0] + Math.PI / 36;
const actuatorResult = await request("/api/motion/actuator", {
  schema_version: 3,
  request_id: "integration-actuator-five-degrees",
  model_revision: model.model_revision,
  actuator_key: actuator.key,
  position_rad: actuatorTarget,
});
assert.equal(actuatorResult.state, "succeeded");
const atActuatorTarget = await waitFor(
  () => snapshot("arm-execution"),
  (state) => state.values.arm_state.actuators_rad[0] > before.actuators_rad[0],
);

const start = model.named_targets.find((target) => target.key === "start");
assert.ok(start, "model publishes the start target");
assert.equal(start.joint_positions_rad.joint3, (-5 * Math.PI) / 180);
const testTarget = model.named_targets.find((target) => target.key === "test");
assert.ok(testTarget, "model publishes the test target");
assert.equal(testTarget.joint_positions_rad.joint3, (-20 * Math.PI) / 180);
assert.equal(
  Object.keys(testTarget.joint_positions_rad).length,
  model.joints.length,
);
assert.equal(testTarget.actuator_positions_rad.gripper, 0);
const startResult = await request("/api/motion/request", {
  schema_version: 3,
  request_id: "integration-start",
  model_revision: model.model_revision,
  joints: model.joints.map((joint) => ({
    joint_key: joint.key,
    position_rad: start.joint_positions_rad[joint.key],
  })),
  actuators: model.tool_actuators.map((item) => ({
    actuator_key: item.key,
    position_rad: start.actuator_positions_rad[item.key],
  })),
  options: {},
  action: "apply",
});
assert.equal(startResult.value.state, "succeeded");
const atStart = await waitFor(
  () => snapshot("arm-execution"),
  (state) =>
    model.joints.every(
      (joint, index) =>
        Math.abs(
          state.values.arm_state.joints_rad[index] -
            start.joint_positions_rad[joint.key],
        ) < 1e-9,
    ) &&
    Math.abs(state.values.arm_state.actuators_rad[0]) <= (2 * Math.PI) / 180,
);

const cancellable = [...atStart.values.arm_state.joints_rad];
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
    position_rad: atStart.values.arm_state.actuators_rad[index],
  })),
  options: {},
  action: "apply",
});
await waitFor(
  () => snapshot("motion"),
  (state) =>
    state.values.motion_state.latest_motion.request_id ===
      "integration-interrupted-motion" &&
    ["planning", "executing"].includes(
      state.values.motion_state.latest_motion.state,
    ),
);
const cancelResult = await request("/api/motion/cancel", {
  schema_version: 3,
  request_id: "integration-cancel",
  action: "cancel",
});
assert.equal(cancelResult.request_id, "integration-cancel");
assert.equal(cancelResult.value.state, "cancelled");
const interruptedResult = await interruptedRequest;
assert.equal(interruptedResult.request_id, "integration-interrupted-motion");
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
assert.equal(simulatedCommand.model_revision, "stararm-102-fl-v1");
assert.equal(simulatedCommand.joints_rad.length, 6);
assert.equal(simulatedCommand.actuators_rad.length, 1);

const execution = await snapshot("arm-execution");
assert.equal(execution.values.transport_state.connected, false);
assert.equal(execution.values.arm_state.feedback_source, "software");
assert.equal(execution.values.execution_service_state.has_input, true);
console.log("software-flow: ok");
