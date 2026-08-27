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

const tracking = await waitFor(
  () => snapshot("tracking"),
  (state) => state.namespace === "tracking",
);
const selectedInput = tracking.values.discovery_state?.selected_source;
if (selectedInput) {
  await request("/api/tracking/source", {
    schema_version: 1,
    request_id: "integration-input-unselect",
    action: "unselect",
    runtime_identity: selectedInput.runtime_identity,
    user_path: selectedInput.user_path,
    interaction_profile: selectedInput.interaction_profile,
    confirmed_host_association:
      tracking.values.discovery_state.confirmed_host_association,
  });
  await waitFor(
    () => snapshot("spatial"),
    (state) => state.values.relative_motion?.active === false,
  );
  await request("/api/tracking/source", {
    schema_version: 1,
    request_id: "integration-input-restore",
    action: "select",
    runtime_identity: selectedInput.runtime_identity,
    user_path: selectedInput.user_path,
    interaction_profile: selectedInput.interaction_profile,
    confirmed_host_association:
      tracking.values.discovery_state.confirmed_host_association,
  });
  await waitFor(
    () => snapshot("tracking"),
    (state) =>
      state.values.discovery_state?.selected_source_id ===
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

await request("/api/arm-execution/disconnect", {
  schema_version: 1,
  request_id: "integration-disconnect",
  action: "disconnect",
  fields: {},
});

const initialSpatial = await snapshot("spatial");
const originalScale =
  initialSpatial.values.spatial_config_state.translation_scale;
const changed = await request(
  "/api/spatial/config",
  {
    schema_version: 1,
    request_id: "integration-scale",
    patch: { translation_scale: 0.25 },
  },
  "PATCH",
);
assert.equal(changed.value.translation_scale, 0.25);
await request(
  "/api/spatial/config",
  {
    schema_version: 1,
    request_id: "integration-scale-restore",
    patch: { translation_scale: originalScale },
  },
  "PATCH",
);

const origin = await request("/api/spatial/origin", {
  schema_version: 1,
  request_id: "integration-origin",
});
if (origin.value === null) {
  assert.equal(typeof origin.original_error, "string");
} else {
  assert.equal(origin.original_error, null);
  assert.equal(origin.value.origin_position_m.length, 3);
}

await request("/api/motion/mode", {
  schema_version: 1,
  request_id: "integration-manual",
  mode: "manual",
});

const beforeBindings = await snapshot("tracking");
const selectedForBindings =
  beforeBindings.values.discovery_state?.selected_source;
const selectedRuntimeSource =
  beforeBindings.values.discovery_state?.runtime_sources?.find(
    (source) => source.source_id === selectedForBindings?.source_id,
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
    component_paths: binding.configured_components,
    invert: binding.invert,
  }));
if (selectedForBindings?.interaction_profile && booleanComponents.length > 0) {
  const applyBindings = (requestId, bindings) =>
    request("/api/tracking/bindings", {
      schema_version: 1,
      request_id: requestId,
      interaction_profile: selectedForBindings.interaction_profile,
      bindings,
    });
  try {
    const booleanResult = await applyBindings("integration-binding-boolean", [
      {
        action: "control_active",
        action_type: "boolean",
        component_paths: [booleanComponents[0].path],
        invert: false,
      },
    ]);
    assert.equal(booleanResult.original_error, null);
    await waitFor(
      () => snapshot("tracking"),
      (state) => {
        const binding = state.values.discovery_state?.bindings?.find(
          (value) => value.action === "control_active",
        );
        return (
          binding?.active === true &&
          binding.bound_sources.includes(booleanComponents[0].path)
        );
      },
    );
    if (booleanComponents.length > 1) {
      const pairResult = await applyBindings("integration-binding-pair", [
        {
          action: "move_left_right",
          action_type: "boolean",
          component_paths: [
            booleanComponents[0].path,
            booleanComponents[1].path,
          ],
          invert: true,
        },
      ]);
      assert.equal(pairResult.original_error, null);
      await waitFor(
        () => snapshot("tracking"),
        (state) => {
          const binding = state.values.discovery_state?.bindings?.find(
            (value) => value.action === "move_left_right",
          );
          return (
            binding?.active === true &&
            booleanComponents.every((component) =>
              binding.bound_sources.includes(component.path),
            )
          );
        },
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
      schema_version: 1,
      request_id: "integration-joint-five-degrees",
      model_revision: model.model_revision,
      joints: model.joints.map((joint, index) => ({
        joint_key: joint.key,
        position_rad: moved[index],
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

const actuator = model.tool_actuators[0];
assert.ok(actuator, "fixture model publishes an actuator");
const actuatorTarget = before.actuators_rad[0] + Math.PI / 36;
const actuatorResult = await request("/api/motion/actuator", {
  schema_version: 1,
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
const startResult = await request("/api/motion/request", {
  schema_version: 1,
  request_id: "integration-start",
  model_revision: model.model_revision,
  joints: model.joints.map((joint) => ({
    joint_key: joint.key,
    position_rad: start.positions_rad[joint.key],
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
            start.positions_rad[joint.key],
        ) < 1e-9,
    ),
);
assert.ok(
  atStart.values.arm_state.actuators_rad[0] > before.actuators_rad[0],
  "the start target must not reset the independently controlled actuator",
);

const cancellable = [...atStart.values.arm_state.joints_rad];
cancellable[0] += Math.PI / 18;
const interruptedRequest = request("/api/motion/request", {
  schema_version: 1,
  request_id: "integration-interrupted-motion",
  model_revision: model.model_revision,
  joints: model.joints.map((joint, index) => ({
    joint_key: joint.key,
    position_rad: cancellable[index],
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
  schema_version: 1,
  request_id: "integration-cancel",
  action: "cancel",
});
assert.equal(cancelResult.request_id, "integration-cancel");
assert.equal(cancelResult.value.state, "cancelled");
const interruptedResult = await interruptedRequest;
assert.equal(interruptedResult.request_id, "integration-interrupted-motion");
assert.equal(interruptedResult.value.state, "cancelled");

await request("/api/motion/actuator", {
  schema_version: 1,
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
await request("/api/motion/mode", {
  schema_version: 1,
  request_id: "integration-relative-restore",
  mode: "relative",
});

const asset = await fetch(
  `${base}/api/motion/assets/${model.visualization.root_path}`,
);
assert.equal(asset.ok, true);
assert.ok((await asset.arrayBuffer()).byteLength > 0);

try {
  const started = await request("/api/tracking/simulation", {
    schema_version: 1,
    request_id: "integration-simulation-start",
    enabled: true,
  });
  assert.equal(started.value.active, true);
  await waitFor(
    () => snapshot("spatial"),
    (state) =>
      state.values.spatial_config_state?.selected_source_id ===
        "simulation:standard-spatial-cycle" &&
      state.values.relative_motion?.active === true &&
      state.values.relative_motion.translation_m.some(
        (component) => Math.abs(component) > 0,
      ),
  );
  const simulationBeforeScan = await snapshot("tracking");
  await new Promise((resolve) => setTimeout(resolve, 1_100));
  const simulationAfterScan = await snapshot("tracking");
  assert.equal(
    simulationAfterScan.values.discovery_state.simulation.active,
    true,
  );
  assert.ok(
    simulationAfterScan.values.absolute_pose.sequence >
      simulationBeforeScan.values.absolute_pose.sequence,
  );
  const simulatedExecution = await snapshot("arm-execution");
  const simulatedCommand =
    simulatedExecution.values.transport_state.last_command;
  assert.equal(simulatedCommand.model_revision, "stararm-102-fl-v1");
  assert.equal(simulatedCommand.joints_rad.length, 6);
  assert.equal(simulatedCommand.actuators_rad.length, 1);
} finally {
  const stopped = await request("/api/tracking/simulation", {
    schema_version: 1,
    request_id: "integration-simulation-stop",
    enabled: false,
  });
  assert.equal(stopped.value.active, false);
  await waitFor(
    () => snapshot("spatial"),
    (state) => state.values.relative_motion?.active === false,
  );
}

const execution = await snapshot("arm-execution");
assert.equal(execution.values.transport_state.connected, false);
assert.equal(execution.values.arm_state.feedback_source, "software");
assert.equal(execution.values.execution_service_state.has_input, true);
console.log("software-flow: ok");
