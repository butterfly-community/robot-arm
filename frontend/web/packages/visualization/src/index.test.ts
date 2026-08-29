import * as THREE from "three";
import { describe, expect, it } from "vitest";

import { mappedOrientation, mappedPosition } from "./index";

const axes = [
  [0, 0, -1],
  [-1, 0, 0],
  [0, 1, 0],
];

function expectSameRotation(
  actual: THREE.Quaternion,
  expected: THREE.Quaternion,
) {
  expect(Math.abs(actual.dot(expected))).toBeCloseTo(1, 10);
}

describe("mappedPosition", () => {
  it("maps tracking movement into forward, left and up exactly once", () => {
    expect(mappedPosition([0, 0, -0.04], [0, 0, 0], axes, 0.5)).toEqual([
      0.02, 0, 0,
    ]);
    expect(mappedPosition([-0.04, 0, 0], [0, 0, 0], axes, 0.5)).toEqual([
      0, 0.02, 0,
    ]);
    expect(mappedPosition([0, 0.04, 0], [0, 0, 0], axes, 0.5)).toEqual([
      0, 0, 0.02,
    ]);
  });
});

describe("mappedOrientation", () => {
  it("keeps an identity device orientation upright after changing basis", () => {
    expectSameRotation(
      mappedOrientation([0, 0, 0, 1], axes),
      new THREE.Quaternion(),
    );
  });

  it("maps a device Y rotation onto the configured up axis", () => {
    const source = new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, 1, 0),
      Math.PI / 2,
    );
    const expected = new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, 0, 1),
      Math.PI / 2,
    );

    expectSameRotation(mappedOrientation(source.toArray(), axes), expected);
  });

  it.each([
    [
      [1, 0, 0],
      [0, -1, 0],
    ],
    [
      [0, 1, 0],
      [0, 0, 1],
    ],
    [
      [0, 0, 1],
      [-1, 0, 0],
    ],
  ])(
    "maps source axis %j onto configured axis %j",
    (sourceAxis, targetAxis) => {
      const source = new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(...(sourceAxis as [number, number, number])),
        Math.PI / 3,
      );
      const expected = new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(...(targetAxis as [number, number, number])),
        Math.PI / 3,
      );

      expectSameRotation(mappedOrientation(source.toArray(), axes), expected);
    },
  );
});
