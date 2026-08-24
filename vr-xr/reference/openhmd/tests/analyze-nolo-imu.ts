type Row = {
  dt: number;
  gyro: number[];
  accel: number[];
  accelError: number;
  accelIgnored: boolean;
  recovery: number;
};

const path = Deno.args[0];
if (!path) {
  console.error(
    "用法: deno run --allow-read vr-xr/tests/analyze-nolo-imu.ts <CSV> [起始行]",
  );
  Deno.exit(2);
}

const startArgument = Deno.args[1] ?? "2";
if (!/^\d+$/.test(startArgument) || Number(startArgument) < 2) {
  console.error(`起始行必须是大于等于 2 的整数，实际为：${startArgument}`);
  Deno.exit(2);
}
const startLine = Number(startArgument);
const text = await Deno.readTextFile(path);
if (!text.trim()) {
  console.error("CSV 文件为空");
  Deno.exit(1);
}
const lines = text.split(/\r?\n/);
const expectedHeader = "time_s,dt_s,gx_rad_s,gy_rad_s,gz_rad_s";
if (!lines[0].startsWith(expectedHeader)) {
  console.error("CSV 表头不是 NOLO_FUSION_LOG 格式");
  Deno.exit(1);
}
if (startLine > lines.length) {
  console.error(`起始行 ${startLine} 超出文件总行数 ${lines.length}`);
  Deno.exit(2);
}

let discardedRows = 0;
const rows: Row[] = lines.slice(startLine - 1).flatMap((line) => {
  if (!line.trim()) return [];
  const v = line.split(",").map(Number);
  if (v.length < 15 || !v.slice(0, 15).every(Number.isFinite)) {
    discardedRows++;
    return [];
  }
  const row = {
    dt: v[1],
    gyro: v.slice(2, 5),
    accel: v.slice(5, 8),
    accelError: v[12],
    accelIgnored: v[13] !== 0,
    recovery: v[14],
  };
  if (row.dt <= 0.002 || row.dt >= 0.03) {
    discardedRows++;
    return [];
  }
  return [row];
});

const norm = (v: number[]) => Math.hypot(...v);
const scale = (v: number[], s: number) => v.map((x) => x * s);
const subtract = (a: number[], b: number[]) => a.map((x, i) => x - b[i]);
const dot = (a: number[], b: number[]) =>
  a.reduce((sum, x, i) => sum + x * b[i], 0);
const cross = (a: number[], b: number[]) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const unit = (v: number[]) => {
  const length = norm(v);
  return length > 1e-9 ? scale(v, 1 / length) : null;
};

function smoothAcceleration(index: number, radius = 2): number[] | null {
  const sum = [0, 0, 0];
  let count = 0;
  for (let i = index - radius; i <= index + radius; i++) {
    if (!rows[i]) continue;
    rows[i].accel.forEach((value, axis) => sum[axis] += value);
    count++;
  }
  return count ? unit(scale(sum, 1 / count)) : null;
}

const permutations = [
  [0, 1, 2],
  [0, 2, 1],
  [1, 0, 2],
  [1, 2, 0],
  [2, 0, 1],
  [2, 1, 0],
];
const signs = [-1, 1];
const axisNames = ["X", "Y", "Z"];

type Candidate = {
  permutation: number[];
  sign: number[];
  name: string;
  determinant: number;
  fittedScale: number;
  relativeRmse: number;
  samples: number;
};

function permutationParity(permutation: number[]): number {
  let inversions = 0;
  for (let i = 0; i < 3; i++) {
    for (let j = i + 1; j < 3; j++) {
      if (permutation[i] > permutation[j]) inversions++;
    }
  }
  return inversions % 2 ? -1 : 1;
}

function evaluate(permutation: number[], sign: number[]): Candidate | null {
  let dp = 0;
  let pp = 0;
  let dd = 0;
  let samples = 0;
  const window = 12;
  for (let i = 3; i < rows.length - window - 3; i += 2) {
    const end = i + window;
    const startAccel = smoothAcceleration(i);
    const endAccel = smoothAcceleration(end);
    if (startAccel === null || endAccel === null) continue;
    const midpointAccel = unit(
      startAccel.map((value, axis) => value + endAccel[axis]),
    );
    if (midpointAccel === null) continue;
    const integratedGyro = [0, 0, 0];
    let usable = true;
    for (let j = i; j < end; j++) {
      const accelMagnitude = norm(rows[j].accel);
      if (accelMagnitude < 8.3 || accelMagnitude > 11.3) {
        usable = false;
        break;
      }
      permutation.forEach((source, axis) => {
        integratedGyro[axis] += sign[axis] * rows[j].gyro[source] * rows[j].dt;
      });
    }
    const angle = norm(integratedGyro);
    if (!usable || angle < 0.025 || angle > 0.5) continue;

    const observed = subtract(endAccel, startAccel);
    const predicted = scale(cross(integratedGyro, midpointAccel), -1);
    dp += dot(observed, predicted);
    pp += dot(predicted, predicted);
    dd += dot(observed, observed);
    samples++;
  }
  if (samples < 30 || pp < 1e-9 || dd < 1e-9) return null;
  const fittedScale = dp / pp;
  if (fittedScale <= 0) return null;
  const squaredError = Math.max(
    0,
    dd - 2 * fittedScale * dp + fittedScale ** 2 * pp,
  );
  return {
    permutation,
    sign,
    name: permutation.map((source, axis) =>
      `${sign[axis] > 0 ? "+" : "-"}${axisNames[source]}`
    ).join(" "),
    determinant: permutationParity(permutation) *
      sign.reduce((a, b) => a * b, 1),
    fittedScale,
    relativeRmse: Math.sqrt(squaredError / dd),
    samples,
  };
}

const candidates: Candidate[] = [];
for (const permutation of permutations) {
  for (const sx of signs) {
    for (const sy of signs) {
      for (const sz of signs) {
        const candidate = evaluate(permutation, [sx, sy, sz]);
        if (candidate) candidates.push(candidate);
      }
    }
  }
}
candidates.sort((a, b) => a.relativeRmse - b.relativeRmse);

const ignored = rows.filter((row) => row.accelIgnored).length;
const maxRecovery = rows.reduce((max, row) => Math.max(max, row.recovery), 0);
const maxAccelError = rows.reduce(
  (max, row) => Math.max(max, row.accelError),
  0,
);
console.log(`有效记录: ${rows.length}`);
console.log(`丢弃记录: ${discardedRows}`);
console.log(
  `加速度被拒绝: ${(100 * ignored / Math.max(1, rows.length)).toFixed(1)}%`,
);
console.log(`最大恢复触发值: ${maxRecovery.toFixed(3)}`);
console.log(`最大加速度方向误差: ${maxAccelError.toFixed(1)}°`);
console.log("\n候选 gyro → accel 轴映射（输出轴依次为 accel X/Y/Z）：");
for (const candidate of candidates.slice(0, 8)) {
  console.log(
    `${candidate.name.padEnd(10)} det=${
      candidate.determinant.toString().padStart(2)
    } ` +
      `比例=${candidate.fittedScale.toFixed(3)} ` +
      `相对误差=${candidate.relativeRmse.toFixed(3)} 样本=${candidate.samples}`,
  );
}

if (!candidates.length) {
  console.error("没有足够的有效旋转样本；请缓慢绕至少两个不同轴旋转手柄。 ");
  Deno.exit(1);
}
