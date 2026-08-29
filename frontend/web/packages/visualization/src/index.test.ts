import * as THREE from "three";
import { describe, expect, it } from "vitest";

import {
  relativeMotionPose,
  robotToSceneOrientation,
  robotToScenePosition,
} from "./index";

function expectSameRotation(
  actual: THREE.Quaternion,
  expected: THREE.Quaternion,
) {
  expect(Math.abs(actual.dot(expected))).toBeCloseTo(1, 10);
}

describe("robotToScenePosition", () => {
  it("renders forward, left and up on the visible front, left and vertical axes", () => {
    expect(robotToScenePosition([0.1, 0.2, 0.3])).toEqual([-0.2, 0.3, 0.1]);
  });
});

describe("robotToSceneOrientation", () => {
  it("keeps identity upright", () => {
    expectSameRotation(
      robotToSceneOrientation([0, 0, 0, 1]),
      new THREE.Quaternion(),
    );
  });

  it("converts robot up rotation into the scene up axis", () => {
    const source = new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, 0, 1),
      Math.PI / 2,
    );
    const expected = new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, -1, 0),
      Math.PI / 2,
    );
    expectSameRotation(robotToSceneOrientation(source.toArray()), expected);
  });
});

describe("relativeMotionPose", () => {
  it("adapts an already transformed motion message without changing translation", () => {
    const pose = relativeMotionPose({
      schema_version: 2,
      sequence: 1,
      source_time_ns: 2,
      transformed_time_ns: 3,
      control_session_id: 4,
      active: true,
      translation_m: [0.1, 0.2, 0.3],
      front_pitch_rad: 0.4,
      horizontal_arc_rad: 0.5,
      actuator_actions: {
        primary_tool_open: {
          is_active: false,
          changed_since_last_sync: false,
          value: false,
        },
        primary_tool: {
          is_active: false,
          changed_since_last_sync: false,
          value: 0,
        },
      },
    });
    expect(pose.position_m).toEqual([0.1, 0.2, 0.3]);
    expectSameRotation(
      new THREE.Quaternion().fromArray(pose.orientation_xyzw),
      new THREE.Quaternion().setFromEuler(new THREE.Euler(0.4, 0.5, 0, "XYZ")),
    );
  });
});
