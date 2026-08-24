const EPSILON = 1e-9;

export function subtract(a, b) {
  return a.map((value, index) => value - b[index]);
}

export function dot(a, b) {
  return a.reduce((sum, value, index) => sum + value * b[index], 0);
}

export function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

export function unit(vector) {
  const length = Math.hypot(...vector);
  if (length < EPSILON) throw new Error("无法归一化零向量");
  return vector.map((value) => value / length);
}

export function normalizeQuaternion(quaternion) {
  const length = Math.hypot(...quaternion);
  if (length < EPSILON) throw new Error("无法归一化零四元数");
  return quaternion.map((value) => value / length);
}

export function conjugateQuaternion([x, y, z, w]) {
  return [-x, -y, -z, w];
}

export function multiplyQuaternions([ax, ay, az, aw], [bx, by, bz, bw]) {
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
}

function median(values) {
  const sorted = [...values].sort((a, b) => a - b);
  const middle = Math.floor(sorted.length / 2);
  return sorted.length % 2
    ? sorted[middle]
    : (sorted[middle - 1] + sorted[middle]) / 2;
}

function medianVector(vectors) {
  return vectors[0].map((_, index) => median(vectors.map((v) => v[index])));
}

function averageQuaternions(quaternions) {
  const anchor = normalizeQuaternion(quaternions[0]);
  const sum = [0, 0, 0, 0];
  for (const input of quaternions) {
    let quaternion = normalizeQuaternion(input);
    if (dot(anchor, quaternion) < 0) quaternion = quaternion.map((v) => -v);
    quaternion.forEach((value, index) => sum[index] += value);
  }
  return normalizeQuaternion(sum);
}

export function averagePoseSamples(samples) {
  if (!samples.length) throw new Error("位姿样本为空");
  const position = medianVector(samples.map((sample) => sample.position));
  const orientation = averageQuaternions(
    samples.map((sample) => sample.orientation),
  );
  const rmsMeters = Math.sqrt(
    samples.reduce((sum, sample) => {
      const delta = subtract(sample.position, position);
      return sum + dot(delta, delta);
    }, 0) / samples.length,
  );
  return { position, orientation, rmsMeters, sampleCount: samples.length };
}

// NOLO's official reference-direction calibration asks the user to face the
// base station and point each tracked device at it. Tracking positions are
// relative to the base-station origin, so that constraint supplies the yaw
// direction which a gyro/accelerometer-only AHRS cannot observe.
export function frameFacingBaseStation(
  position,
  basePosition = [0, 0, 0],
) {
  const toBase = subtract(basePosition, position);
  const horizontalToBase = [toBase[0], 0, toBase[2]];
  const distance = Math.hypot(horizontalToBase[0], horizontalToBase[2]);
  const forward = unit(horizontalToBase);
  const up = [0, 1, 0];
  const right = unit(cross(forward, up));
  return {
    forward,
    right,
    up,
    distance,
  };
}

export function transformPosition(position, originPosition, frame) {
  const delta = subtract(position, originPosition);
  if (!frame) return delta;
  return [
    dot(delta, frame.right),
    dot(delta, frame.up),
    -dot(delta, frame.forward),
  ];
}

export function relativeQuaternionToModel(reference, current) {
  const rawRelative = normalizeQuaternion(
    multiplyQuaternions(conjugateQuaternion(reference), current),
  );
  // Controlled controller motion establishes this proper body-axis rotation:
  // head-up raw -X -> model +X, left-tilt raw +Y -> model +Z,
  // and left-yaw raw +Z -> model +Y.
  let model = [
    -rawRelative[0],
    rawRelative[2],
    rawRelative[1],
    rawRelative[3],
  ];
  if (model[3] < 0) model = model.map((value) => -value);
  return model;
}

export function quaternionAxisAngle(quaternion) {
  const normalized = normalizeQuaternion(quaternion);
  const w = Math.max(-1, Math.min(1, normalized[3]));
  const halfAngle = Math.acos(w);
  const sinHalfAngle = Math.sin(halfAngle);
  return {
    axis: sinHalfAngle < 1e-6
      ? [0, 0, 0]
      : normalized.slice(0, 3).map((value) => value / sinHalfAngle),
    degrees: 2 * halfAngle * 180 / Math.PI,
  };
}
