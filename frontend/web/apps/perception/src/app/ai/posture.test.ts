import { describe, expect, it } from "vitest";
import type { ArmState, MotionState, RobotModelInfo } from "@robot/contracts";
import { gripperParameters, jointReadings, postureSummary } from "./posture";

const model = {
  model_revision: "test",
  base_frame: "base",
  tcp_frame: "tcp",
  joints: [{ key: "j1", label: "J1", minimum: -Math.PI, maximum: Math.PI }],
  tool_actuators: [
    {
      key: "gripper",
      label: "夹爪",
      minimum: 0,
      maximum: Math.PI / 2,
      unit: "rad",
    },
  ],
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
  it("converts model-owned opening references without copying or inventing a calibration curve", () => {
    const reference = {
      source: "fixture/graspgenx/manifest.json",
      frame: "gripper_base",
      samples: [
        {
          position_rad: Math.PI / 6,
          inner_width_m: 0.071,
          reference_center_m: [0.00134, 0.00015, 0.08] as [
            number,
            number,
            number,
          ],
        },
      ],
    };
    const withReference = {
      ...model,
      tool_actuators: [
        { ...model.tool_actuators[0], opening_reference: reference },
      ],
    };
    const result = gripperParameters(withReference)[0];
    expect(result.range_deg).toEqual([0, 90]);
    expect(result.opening_reference?.source).toBe(reference.source);
    expect(result.opening_reference?.samples).toHaveLength(1);
    expect(result.opening_reference?.samples[0].inner_width_mm).toBe(71);
    expect(result.opening_reference?.samples[0].position_deg).toBeCloseTo(30);
    expect(reference.samples[0]).not.toHaveProperty("position_deg");
    expect(gripperParameters(model)[0].opening_reference).toBeUndefined();
  });
  it("reports intermediate gripper travel from actual feedback and model limits, without inventing gap width", () => {
    const summary = postureSummary(
      model,
      { ...arm, actuators_rad: [Math.PI / 6] },
      motion,
    );
    const gripper = summary.actual!.gripper[0];
    expect(gripper.position_deg).toBeCloseTo(30);
    expect(gripper.range_deg).toEqual([0, 90]);
    expect(gripper.travel_percent).toBeCloseTo(100 / 3);
    expect(gripper).not.toHaveProperty("width_mm");
    for (const value of [0, Math.PI / 4, Math.PI / 2]) {
      expect(
        postureSummary(model, { ...arm, actuators_rad: [value] }).actual!
          .gripper[0].travel_percent,
      ).toBeCloseTo((value / (Math.PI / 2)) * 100);
    }
    const other = {
      ...model,
      tool_actuators: [{ ...model.tool_actuators[0], minimum: -1, maximum: 1 }],
    };
    expect(postureSummary(other, arm).actual!.gripper[0].travel_percent).toBe(
      50,
    );
    expect(
      postureSummary(model, { ...arm, actuators_rad: [] }).actual!.gripper[0]
        .travel_percent,
    ).toBeNull();
    expect(
      postureSummary(model, { ...arm, actuators_rad: [NaN] }).actual!.gripper[0]
        .travel_percent,
    ).toBeNull();
  });
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
