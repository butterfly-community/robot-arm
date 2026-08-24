import {
  averagePoseSamples,
  dot,
  frameFacingBaseStation,
  quaternionAxisAngle,
  relativeQuaternionToModel,
  transformPosition,
} from "../src/controller-viewer/public/pose-math.js";

function assert(condition: boolean, message: string) {
  if (!condition) throw new Error(message);
}

function assertNear(actual: number, expected: number, tolerance = 1e-6) {
  assert(
    Math.abs(actual - expected) <= tolerance,
    `expected ${expected} ± ${tolerance}, got ${actual}`,
  );
}

Deno.test("origin pose uses a robust median position", () => {
  const samples = [
    { position: [1.000, 2.000, 3.000], orientation: [0, 0, 0, 1] },
    { position: [1.002, 1.999, 2.998], orientation: [0, 0, 0, 1] },
    { position: [0.999, 2.001, 3.001], orientation: [0, 0, 0, -1] },
    { position: [9.000, 9.000, 9.000], orientation: [0, 0, 0, 1] },
    { position: [1.001, 2.002, 2.999], orientation: [0, 0, 0, 1] },
  ];
  const result = averagePoseSamples(samples);
  assertNear(result.position[0], 1.001);
  assertNear(result.position[1], 2.001);
  assertNear(result.position[2], 3.000);
  assertNear(Math.abs(result.orientation[3]), 1);
});

Deno.test("official NOLO calibration points forward toward base station", () => {
  const frame = frameFacingBaseStation([1.5, 0.8, 2.0]);
  assertNear(frame.forward[0], -0.6);
  assertNear(frame.forward[1], 0);
  assertNear(frame.forward[2], -0.8);
  assertNear(frame.right[0], 0.8);
  assertNear(frame.right[1], 0);
  assertNear(frame.right[2], -0.6);
  assertNear(frame.distance, 2.5);
  assertNear(dot(frame.forward, frame.right), 0);
});

Deno.test("base-station origin cannot define a horizontal direction", () => {
  let rejected = false;
  try {
    frameFacingBaseStation([0.0, 1.0, 0.0]);
  } catch {
    rejected = true;
  }
  assert(rejected, "zero horizontal direction was accepted");
});

Deno.test("human position frame is right +X, up +Y, forward -Z", () => {
  const frame = {
    right: [1, 0, 0],
    up: [0, 1, 0],
    forward: [0, 0, -1],
  };
  const transformed = transformPosition([1.2, 2.3, 2.6], [1, 2, 3], frame);
  assertNear(transformed[0], 0.2);
  assertNear(transformed[1], 0.3);
  assertNear(transformed[2], -0.4);
});

Deno.test("NOLO local +Z rotation maps to model +Y", () => {
  const half = Math.PI / 4;
  const relative = relativeQuaternionToModel(
    [0, 0, 0, 1],
    [0, 0, Math.sin(half), Math.cos(half)],
  );
  assertNear(relative[0], 0);
  assertNear(relative[1], Math.sin(half));
  assertNear(relative[2], 0);
  const axisAngle = quaternionAxisAngle(relative);
  assertNear(axisAngle.axis[1], 1);
  assertNear(axisAngle.degrees, 90);
});

Deno.test("relative orientation removes a non-identity reference", () => {
  const referenceHalfAngle = 15 * Math.PI / 180;
  const currentHalfAngle = 45 * Math.PI / 180;
  const relative = relativeQuaternionToModel(
    [0, 0, Math.sin(referenceHalfAngle), Math.cos(referenceHalfAngle)],
    [0, 0, Math.sin(currentHalfAngle), Math.cos(currentHalfAngle)],
  );
  const axisAngle = quaternionAxisAngle(relative);
  assertNear(axisAngle.axis[0], 0);
  assertNear(axisAngle.axis[1], 1);
  assertNear(axisAngle.axis[2], 0);
  assertNear(axisAngle.degrees, 60);
});

Deno.test("NOLO physical head-up raw -X maps to model +X", () => {
  const half = Math.PI / 4;
  const relative = relativeQuaternionToModel(
    [0, 0, 0, 1],
    [-Math.sin(half), 0, 0, Math.cos(half)],
  );
  const axisAngle = quaternionAxisAngle(relative);
  assertNear(axisAngle.axis[0], 1);
  assertNear(axisAngle.axis[1], 0);
  assertNear(axisAngle.axis[2], 0);
  assertNear(axisAngle.degrees, 90);
});

Deno.test("NOLO physical left-tilt raw +Y maps to model +Z", () => {
  const half = Math.PI / 4;
  const relative = relativeQuaternionToModel(
    [0, 0, 0, 1],
    [0, Math.sin(half), 0, Math.cos(half)],
  );
  const axisAngle = quaternionAxisAngle(relative);
  assertNear(axisAngle.axis[0], 0);
  assertNear(axisAngle.axis[1], 0);
  assertNear(axisAngle.axis[2], 1);
  assertNear(axisAngle.degrees, 90);
});
