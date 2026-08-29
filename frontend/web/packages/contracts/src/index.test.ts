import { describe, expect, it } from "vitest";

import {
  actionGroupOrder,
  feedbackActionCatalog,
  inputActionCatalog,
} from "./index";

const expectedInputActions = [
  "move_forward_back",
  "move_left_right",
  "move_up_down",
  "tool_pitch",
  "tool_yaw",
  "tool_roll",
  "front_pitch",
  "horizontal_arc",
  "primary_tool_open",
  "primary_tool",
  "start_stop",
  "emergency_stop",
  "tool_axis_translation",
  "tool_helical_motion",
] as const;

describe("action catalog", () => {
  it("contains each of the fourteen input actions exactly once", () => {
    const actual = inputActionCatalog.map((action) => action.key);
    expect(actual).toHaveLength(14);
    expect(new Set(actual).size).toBe(14);
    expect([...actual].sort()).toEqual([...expectedInputActions].sort());
  });

  it("uses the requested six groups and one feedback output", () => {
    expect(actionGroupOrder).toEqual([
      "tcp",
      "arc",
      "gripper",
      "control",
      "feedback",
      "compound",
    ]);
    expect(feedbackActionCatalog).toHaveLength(1);
    expect(feedbackActionCatalog[0].item).toBe("primary_tool_feedback");
    for (const group of actionGroupOrder.filter(
      (candidate) => candidate !== "feedback",
    )) {
      expect(
        inputActionCatalog.some((definition) => definition.group === group),
      ).toBe(true);
    }
  });

  it("declares source-arbitration domains once per action", () => {
    expect(
      Object.fromEntries(
        inputActionCatalog.map(({ key, domains }) => [key, domains]),
      ),
    ).toMatchObject({
      move_forward_back: ["position"],
      tool_pitch: ["orientation"],
      primary_tool: [],
      start_stop: [],
      tool_axis_translation: ["position"],
      tool_helical_motion: ["position", "orientation"],
    });
  });
});
