// Formal software chain: generated input -> spatial -> motion -> execution.
// Deliberately refuses to run with a connected physical actuator.
import assert from "node:assert/strict";
import { mkdir, readFile, writeFile } from "node:fs/promises";

const base = process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765";
async function state(namespace) {
  const response = await fetch(`${base}/api/${namespace}/state`);
  assert.ok(response.ok);
  return (await response.json()).values;
}
async function post(path, body) {
  const response = await fetch(`${base}/api/${path}`, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({
      schema_version: 3,
      request_id: crypto.randomUUID(),
      ...body,
    }),
  });
  const result = await response.json();
  assert.ok(response.ok && !result.original_error, JSON.stringify(result));
  return result;
}
async function until(read, accept) {
  const deadline = Date.now() + 15000;
  while (Date.now() < deadline) {
    const value = await read();
    if (accept(value)) return value;
    await new Promise((resolve) => setTimeout(resolve, 20));
  }
  assert.fail("software state did not converge");
}
const before = await state("arm-execution");
assert.equal(
  before.transport_state.connected,
  false,
  "software acceptance only",
);
assert.equal(before.arm_state.feedback_source, "software");
const previousMode = (await state("motion")).motion_state.control_mode;
const configPath = new URL(
  "../../backend/config/runtime/controller-input.json",
  import.meta.url,
);
const configBefore = await readFile(configPath, "utf8");
const model = before.robot_model_info;
const work = model.named_targets.find((target) => target.key === "work");
assert.ok(work);
async function move(joints, actuators) {
  await post("motion/mode", { mode: "manual" });
  await post("motion/request", {
    action: "apply",
    model_revision: model.model_revision,
    joints: Object.entries(joints).map(([joint_key, position_rad]) => ({
      joint_key,
      position_rad,
    })),
    actuators: Object.entries(actuators).map(
      ([actuator_key, position_rad]) => ({ actuator_key, position_rad }),
    ),
    options: {},
  });
}
const results = [];
try {
  await move(work.joint_positions_rad, work.actuator_positions_rad);
  const armStart = (await state("arm-execution")).arm_state.joints_rad;
  for (const [item, field, axis] of [
    ["move_up_down", "translation_m", 2],
    ["move_forward_back", "translation_m", 0],
    ["move_left_right", "translation_m", 1],
    ["tool_pitch", "tool_pitch_rad"],
    ["tool_yaw", "tool_yaw_rad"],
    ["tool_roll", "tool_roll_rad"],
  ]) {
    for (const value of [1, -1]) {
      await post("motion/mode", { mode: "relative" });
      await post("tracking/simulation", { enabled: true, item, value });
      const mapped = await until(
        () => state("spatial"),
        (state) => {
          const frame = state.transformed_control;
          const component =
            axis === undefined ? frame?.[field] : frame?.[field]?.[axis];
          return frame?.active && component * value > 0;
        },
      );
      const tracking = await until(
        () => state("tracking"),
        (state) =>
          state.discovery_state.live_component_values[
            "simulation:generic-action-source"
          ]?.[`action/${item}`] === value &&
          state.control_input[item].value * value > 0,
      );
      // The normal binding path applies the existing One Euro input filter.
      assert.ok(tracking.control_input[item].value * value > 0);
      assert.equal(
        tracking.discovery_state.live_component_values[
          "simulation:generic-action-source"
        ][`action/${item}`],
        value,
      );
      const motion = await until(
        () => state("motion"),
        (state) => state.motion_state.target_tool_pose != null,
      );
      results.push({
        item,
        value,
        mapped: mapped.transformed_control,
        target: motion.motion_state.target_tool_pose,
      });
      // Allow software Servo/execution feedback to consume the target as well.
      await new Promise((resolve) => setTimeout(resolve, 1000));
      results.at(-1).feedback = (await state("arm-execution")).arm_state;
      results.at(-1).motion = (await state("motion")).motion_state;
      console.log(
        item,
        value,
        JSON.stringify(results.at(-1).feedback.joints_rad),
        JSON.stringify(results.at(-1).motion.diagnostics),
      );
      await post("tracking/simulation", { enabled: false, item: null });
      await until(
        () => state("spatial"),
        (state) => state.transformed_control?.active === false,
      );
      await until(
        () => state("tracking"),
        (state) => state.discovery_state.simulation.active === false,
      );
    }
  }
  const armEnd = (await state("arm-execution")).arm_state.joints_rad;
  assert.ok(
    results.some(({ feedback }) =>
      feedback.joints_rad.some(
        (value, index) => Math.abs(value - armStart[index]) > 1e-6,
      ),
    ),
    "software arm received motion",
  );
  assert.equal(
    await readFile(configPath, "utf8"),
    configBefore,
    "mouse input must not persist bindings",
  );
  await mkdir(new URL("../../temp/mouse-control/", import.meta.url), {
    recursive: true,
  });
  await writeFile(
    new URL("../../temp/mouse-control/software-chain.json", import.meta.url),
    JSON.stringify({ armStart, armEnd, results }, null, 2),
  );
  console.log(
    `Verified ${results.length} signed inputs through spatial and motion; software feedback changed; bindings unchanged.`,
  );
} finally {
  await post("tracking/simulation", { enabled: false, item: null });
  await move(
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
  );
  await post("motion/mode", { mode: previousMode });
}
