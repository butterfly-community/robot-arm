import { describe, expect, it } from "vitest";
import { tcpTargetPreview } from "./robot-viewer";

describe("TCP target preview", () => {
  it("distinguishes tool-axis displacement from base-axis displacement", () => {
    const current = {
      frame: "base",
      position_m: [0.1, 0.2, 0.3] as [number, number, number],
      orientation_xyzw: [0, 0, Math.SQRT1_2, Math.SQRT1_2] as [
        number,
        number,
        number,
        number,
      ],
    };
    const target = {
      relative: true,
      pose: {
        frame: "tcp",
        position_m: [0.02, 0, 0] as [number, number, number],
        orientation_xyzw: [0, 0, 0, 1] as [number, number, number, number],
      },
    };
    const tool = tcpTargetPreview(current, target, "base", "tcp");
    expect(tool.position_m[0]).toBeCloseTo(0.1, 12);
    expect(tool.position_m[1]).toBeCloseTo(0.22, 12);
    target.pose.frame = "base";
    expect(
      tcpTargetPreview(current, target, "base", "tcp").position_m[0],
    ).toBeCloseTo(0.12, 12);
    target.relative = false;
    expect(tcpTargetPreview(current, target, "base", "tcp").position_m).toEqual(
      [0.02, 0, 0],
    );
  });
});
