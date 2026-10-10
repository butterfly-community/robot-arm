import { describe, expect, it } from "vitest";
import type { ArmState, MotionState, RobotModelInfo } from "@robot/contracts";
import { jointReadings, postureSummary } from "./posture";

const model = {
  model_revision: "test",
  base_frame: "base",
  tcp_frame: "tcp",
  joints: [{ key: "j1", label: "J1", minimum: -Math.PI, maximum: Math.PI }],
  tool_actuators: [{ key: "gripper", label: "夹爪" }],
} as RobotModelInfo;
const arm: ArmState = {
  model_revision: "test",
  feedback_source: "hardware",
  sequence: 12,
  sample_time_ns: 1234,
  joints_rad: [Math.PI / 2],
  actuators_rad: [0],
};
const motion: MotionState = {
  control_mode: "manual",
  current_tool_pose: {
    frame: "base",
    position_m: [0.1, 0.2, 0.3],
    orientation_xyzw: [0, 0, 0, 1],
    arm_state: { ...arm, sequence: 11, joints_rad: [0] },
  },
  target_tool_pose: {
    frame: "base",
    position_m: [0.4, 0.5, 0.6],
    orientation_xyzw: [0, 0, 1, 0],
  },
};

describe("AI actual posture and units", () => {
  it("keeps actual angles, FK sample and commanded target distinct", () => {
    const summary = postureSummary(model, arm, motion);
    expect(summary.actual?.sequence).toBe(12);
    expect(summary.actual?.joints[0].position_deg).toBe(90);
    expect(summary.measured_tcp?.feedback.sequence).toBe(11);
    expect(summary.measured_tcp?.feedback.joints[0].position_deg).toBe(0);
    expect(summary.measured_tcp?.position_mm).toEqual([100, 200, 300]);
    expect(summary.commanded_tcp_not_measured?.position_mm).toEqual([
      400, 500, 600,
    ]);
    expect(summary.actual?.gripper[0].position_deg).toBe(0);
  });
  it("does not replace absent or wrong-model feedback with commanded values", () => {
    expect(postureSummary().available).toBe(false);
    const summary = postureSummary(
      model,
      { ...arm, model_revision: "other" },
      {
        ...motion,
        current_tool_pose: {
          ...motion.current_tool_pose!,
          arm_state: { ...arm, model_revision: "other" },
        },
      },
    );
    expect(summary.available).toBe(false);
    expect(summary.actual).toBeNull();
    expect(summary.measured_tcp).toBeNull();
    expect(summary.commanded_tcp_not_measured).not.toBeNull();
  });
  it("represents missing and non-finite angles as unavailable, not zero", () => {
    for (const positions of [[], [NaN], [Infinity]]) {
      expect(jointReadings(model, positions)[0]).toMatchObject({
        position_rad: null,
        position_deg: null,
      });
    }
    expect(jointReadings(model, [0])[0].position_deg).toBe(0);
  });
});
