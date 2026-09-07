import assert from "node:assert/strict";
import { mkdir, writeFile } from "node:fs/promises";

// Formal HTTP → Dora → MoveIt → software execution regression. No device opens.
const base = process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765";
const results = [];
const id = () => `audit-${crypto.randomUUID()}`;
async function state(namespace) {
  const response = await fetch(`${base}/api/${namespace}/state`);
  assert.ok(response.ok);
  return (await response.json()).values;
}
async function post(path, body) {
  const started = performance.now();
  const response = await fetch(`${base}/api/${path}`, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ schema_version: 3, request_id: id(), ...body }),
    signal: AbortSignal.timeout(120_000),
  });
  const result = { status: response.status, body: await response.json() };
  console.log(
    path,
    body.action ?? body.mode ?? "",
    body.request_id ?? "",
    Math.round(performance.now() - started),
    result.status,
    result.body.value?.state ?? result.body.original_error ?? "ok",
  );
  return result;
}
async function ok(path, body) {
  const response = await post(path, body);
  assert.ok(response.status < 300, JSON.stringify(response));
  assert.equal(
    response.body.original_error ?? null,
    null,
    JSON.stringify(response),
  );
  return response.body;
}
async function until(read, accept) {
  const deadline = Date.now() + 30_000;
  let value;
  while (Date.now() < deadline) {
    value = await read();
    if (accept(value)) return value;
    await new Promise((resolve) => setTimeout(resolve, 20));
  }
  assert.fail(`State did not converge: ${JSON.stringify(value)}`);
}
const before = await until(
  () => state("arm-execution"),
  (value) => value.robot_model_info && value.arm_state,
);
assert.equal(before.transport_state.selected_endpoint, null);
assert.equal(before.transport_state.connected, false);
assert.equal(before.arm_state.feedback_source, "software");
const previousMode = (await state("motion")).motion_state.control_mode;
const model = before.robot_model_info;
const work = model.named_targets.find((target) => target.key === "work");
assert.ok(work);
const target = (joints, actuators, options = {}) => ({
  action: "apply",
  model_revision: model.model_revision,
  joints: Object.entries(joints).map(([joint_key, position_rad]) => ({
    joint_key,
    position_rad,
  })),
  actuators: Object.entries(actuators).map(([actuator_key, position_rad]) => ({
    actuator_key,
    position_rad,
  })),
  options,
});
try {
  for (const [path, body] of [
    [
      "perception/camera",
      { action: "select", output_frames_per_second: "invalid" },
    ],
    ["tracking/simulation", { enabled: "invalid" }],
    ["motion/mode", { mode: "invalid" }],
    ["motion/request", { action: "apply", joints: "invalid" }],
  ]) {
    const response = await post(path, body);
    assert.equal(response.status, 400, JSON.stringify(response));
    results.push({
      test: "invalid request stays at gateway",
      path,
      ...response,
    });
  }
  await ok("motion/mode", { mode: "manual" });
  await ok(
    "motion/request",
    target(work.joint_positions_rad, work.actuator_positions_rad),
  );
  const requestId = id();
  const movingTarget = {
    ...work.joint_positions_rad,
    [model.joints[0].key]: 0.8,
  };
  const moving = post("motion/request", {
    ...target(movingTarget, work.actuator_positions_rad, {
      velocity_scaling: 0.03,
    }),
    request_id: requestId,
  });
  await until(
    () => state("motion"),
    (value) =>
      value.motion_status.request_id === requestId &&
      value.motion_status.state === "executing",
  );
  // Observe real software joint progress, not just the pre-execution status.
  await until(
    () => state("arm-execution"),
    (value) =>
      Math.abs(
        value.arm_state.joints_rad[0] -
          work.joint_positions_rad[model.joints[0].key],
      ) >
      Math.abs(
        movingTarget[model.joints[0].key] -
          work.joint_positions_rad[model.joints[0].key],
      ) *
        0.1,
  );
  const duplicate = await post("motion/request", {
    ...target(movingTarget, work.actuator_positions_rad),
    request_id: requestId,
  });
  assert.ok(duplicate.body.original_error, "duplicate pending ID rejected");
  const rejected = await post("motion/request", {
    ...target({}, {}),
    model_revision: "not-the-current-model",
  });
  assert.ok(rejected.body.original_error);
  assert.equal((await state("motion")).motion_status.request_id, requestId);
  await ok("motion/request", { action: "cancel" });
  const cancelled = await moving;
  assert.equal(cancelled.body.request_id, requestId);
  assert.equal(
    cancelled.body.value.state,
    "cancelled",
    JSON.stringify(cancelled),
  );
  const stoppedAt = (await state("arm-execution")).arm_state.joints_rad[0];
  assert.ok(
    Math.abs(stoppedAt - movingTarget[model.joints[0].key]) > 1e-6,
    "cancel stopped feedback before the requested final position",
  );
  await ok(
    "motion/request",
    target(work.joint_positions_rad, work.actuator_positions_rad),
  );
  results.push({
    test: "real ROS cancel, ID ownership, next request recovery",
    cancelled,
    stoppedAt,
    duplicate,
    rejected,
  });

  await ok("motion/mode", { mode: "perception" });
  const actuator = await post("motion/actuator", {
    model_revision: model.model_revision,
    actuator_key: model.tool_actuators[0].key,
    position_rad: 0,
  });
  assert.ok(actuator.body.original_error, JSON.stringify(actuator));
  results.push({ test: "actuator mode failure is returned", actuator });
  const stalePick = await post("perception/pick-place", {
    scene_sequence: 0,
    object_id: "stale",
    placement_region_id: "stale",
  });
  assert.equal(stalePick.status, 400, JSON.stringify(stalePick));
  assert.ok(stalePick.body.original_error);
  results.push({ test: "pick-place rejection is not a false 202", stalePick });
  assert.equal((await state("arm-execution")).transport_state.connected, false);
} finally {
  await ok("motion/request", { action: "cancel" });
  await ok("motion/mode", { mode: "manual" });
  await ok(
    "motion/request",
    target(
      Object.fromEntries(
        model.joints.map((joint, index) => [
          joint.key,
          before.arm_state.joints_rad[index],
        ]),
      ),
      Object.fromEntries(
        model.tool_actuators.map((actuator, index) => [
          actuator.key,
          before.arm_state.actuators_rad[index],
        ]),
      ),
    ),
  );
  await ok("motion/mode", { mode: previousMode });
  await mkdir(new URL("../../temp/repository-audit/", import.meta.url), {
    recursive: true,
  });
  await writeFile(
    new URL("../../temp/repository-audit/runtime.json", import.meta.url),
    JSON.stringify(results, null, 2),
  );
}
console.log(`Passed ${results.length} repository audit regression groups.`);
