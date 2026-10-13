"use client";
import { useEffect, useState } from "react";
import { useDraftValue } from "@robot/gateway-client";
import { Button, Card, Disclosure, Field, Input, KeyValue } from "@robot/ui";
import { CalibrationValues } from "./calibration-values";

type Result = {
  sources: [string, string];
  board: { square_size_m: number; marker_size_m: number };
  optics: {
    width: number;
    height: number;
    left_rms_px: number;
    right_rms_px: number;
    stereo_rms_px: number;
    sample_count: number;
    left_to_right: { position_m: number[]; orientation_xyzw: number[] };
    left_k: number[];
    right_k: number[];
    left_d: number[];
    right_d: number[];
  };
  camera_pose?: { position_m: number[]; orientation_xyzw: number[] } | null;
  translation_residuals_m: number[];
  rotation_residuals_rad: number[];
};
type Session = {
  busy: boolean;
  message: string;
  original_error?: string;
  sample_count: number;
  pose_sample_count: number;
  pair_delta_ms?: number;
  candidate?: Result;
  applied?: Result;
};
type Rig = "external" | "wrist";

export function StereoCalibrations() {
  const [state, setState] = useState<Partial<Record<Rig, Session>>>({});
  const [error, setError] = useState<string>();
  useEffect(() => {
    let stopped = false;
    let timer: ReturnType<typeof setTimeout>;
    const poll = async () => {
      try {
        const response = await fetch("/api/camera/stereo", {
          cache: "no-store",
        });
        if (!response.ok)
          throw Error(`双目标定状态读取失败：HTTP ${response.status}`);
        const value = await response.json();
        if (!stopped) {
          setState(value);
          setError(undefined);
        }
      } catch (e) {
        if (!stopped) setError(String(e));
      } finally {
        if (!stopped) timer = setTimeout(poll, 1000);
      }
    };
    void poll();
    return () => {
      stopped = true;
      clearTimeout(timer);
    };
  }, []);
  return (
    <>
      {(["external", "wrist"] as const).map((rig) => (
        <Calibration
          key={rig}
          rig={rig}
          session={state[rig]}
          connectionError={error}
        />
      ))}
    </>
  );
}

function Calibration({
  rig,
  session,
  connectionError,
}: {
  rig: Rig;
  session?: Session;
  connectionError?: string;
}) {
  const title = rig === "external" ? "外部双目标定" : "腕部双目标定";
  const [pending, setPending] = useState(false);
  const [error, setError] = useState<string>();
  const [depth, setDepth] = useState<Record<string, number | boolean>>();
  const [square, setSquare] = useDraftValue(
    String((session?.applied?.board.square_size_m ?? 0.015) * 1000),
  );
  const [marker, setMarker] = useDraftValue(
    String((session?.applied?.board.marker_size_m ?? 0.011) * 1000),
  );
  const act = async (action: string) => {
    setPending(true);
    setError(undefined);
    setDepth(undefined);
    try {
      const response = await fetch("/api/camera/stereo", {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify({
          rig,
          action,
          board: {
            pattern: "charuco",
            dictionary: "DICT_4X4_50",
            squares_x: 5,
            squares_y: 5,
            square_size_m: Number(square) / 1000,
            marker_size_m: Number(marker) / 1000,
            measured_width_m: (Number(square) * 5) / 1000,
            measured_height_m: (Number(square) * 5) / 1000,
          },
        }),
      });
      const value = await response.json();
      if (!response.ok)
        throw Error(value.original_error ?? `HTTP ${response.status}`);
    } catch (e) {
      setError(String(e));
    } finally {
      setPending(false);
    }
  };
  const busy = pending || session?.busy;
  const result = session?.candidate ?? session?.applied;
  return (
    <Card
      className="span-6 stereo-calibration"
      title={title}
      defaultOpen={false}
    >
      <p className="status">
        先绑定并开启同组左右摄像头，保持相对位置固定。复用现有 ChArUco
        板。没有真实摄像头时可选择测试画面；测试结果不适用于真实设备。
      </p>
      <Disclosure title="标定板参数" defaultOpen={false}>
        <div className="form-grid">
          <Field label="方格边长（mm）">
            <Input
              type="number"
              value={square}
              onChange={(e) => setSquare(e.target.value)}
              disabled={busy}
            />
          </Field>
          <Field label="标记边长（mm）">
            <Input
              type="number"
              value={marker}
              onChange={(e) => setMarker(e.target.value)}
              disabled={busy}
            />
          </Field>
        </div>
        <p className="status">
          5 × 5 方格，DICT_4X4_50。填写实际打印尺寸；采样期间保持参数不变。
        </p>
      </Disclosure>
      <section className="camera-binding">
        <h3>1. 内参与双目标定</h3>
        <p className="status">
          移动并倾斜标定板，让两台相机同时看到；每次停稳后采样，覆盖画面不同位置和距离。测试画面需为左右选择同编号
          ChArUco 姿态。
        </p>
        <div className="card-actions">
          <Button disabled={busy} onClick={() => act("sample")}>
            采集双目样本
          </Button>
          <Button
            disabled={busy}
            variant="outline"
            onClick={() => act("solve")}
          >
            求解双目参数
          </Button>
        </div>
        <KeyValue label="本轮双目样本" value={session?.sample_count ?? 0} />
        <KeyValue
          label="左右接收时间差"
          value={
            session?.pair_delta_ms == null
              ? "—"
              : `${session.pair_delta_ms.toFixed(2)} ms（软件配对，非硬件同步）`
          }
        />
      </section>
      <section className="camera-binding">
        <h3>2. 机械臂外参</h3>
        <p className="status">
          {rig === "external"
            ? "标定板固定在夹爪，改变机械臂姿态；相机保持不动。"
            : "双目相机固定在腕部，标定板固定在工作区；改变机械臂位置及不同轴的旋转。"}
          每个姿态停稳后采样，使用电机实际反馈计算的末端位姿。只重复工作位不足以求解手眼关系。
        </p>
        <div className="card-actions">
          <Button
            disabled={busy}
            variant="outline"
            onClick={() => act("sample_pose")}
          >
            采集外参样本
          </Button>
          <Button
            disabled={busy}
            variant="outline"
            onClick={() => act("solve_pose")}
          >
            求解机械臂外参
          </Button>
        </div>
        <KeyValue
          label="本轮外参样本"
          value={session?.pose_sample_count ?? 0}
        />
      </section>
      {result && (
        <section className="camera-binding">
          <h3>{session?.candidate ? "待应用结果" : "上次应用结果"}</h3>
          <KeyValue
            label="尺寸"
            value={`${result.optics.width} × ${result.optics.height}`}
          />
          <KeyValue
            label="求解样本（双目 / 外参）"
            value={`${result.optics.sample_count} / ${result.translation_residuals_m.length}`}
          />
          <KeyValue
            label="左 / 右 / 双目 RMS"
            value={`${result.optics.left_rms_px.toFixed(3)} / ${result.optics.right_rms_px.toFixed(3)} / ${result.optics.stereo_rms_px.toFixed(3)} px`}
          />
          <KeyValue
            label="基线"
            value={`${(Math.hypot(...result.optics.left_to_right.position_m) * 1000).toFixed(2)} mm`}
          />
          <KeyValue
            label="机械臂定位"
            value={
              result.camera_pose
                ? "外参已求解"
                : "尚缺机械臂外参；可生成相机坐标深度"
            }
          />
          {result.translation_residuals_m.length > 0 && (
            <KeyValue
              label="外参最大平移残差"
              value={`${(Math.max(...result.translation_residuals_m) * 1000).toFixed(2)} mm`}
            />
          )}
          {result.rotation_residuals_rad.length > 0 && (
            <KeyValue
              label="外参最大旋转残差"
              value={`${((Math.max(...result.rotation_residuals_rad) * 180) / Math.PI).toFixed(3)}°`}
            />
          )}
          <Disclosure title="完整标定参数" defaultOpen={false}>
            <CalibrationValues label="完整标定参数">
              <KeyValue label="左右来源" value={result.sources.join(" / ")} />
              <KeyValue
                label="左内参 K（行优先）"
                value={result.optics.left_k
                  .map((v) => v.toFixed(4))
                  .join(" / ")}
              />
              <KeyValue
                label="右内参 K（行优先）"
                value={result.optics.right_k
                  .map((v) => v.toFixed(4))
                  .join(" / ")}
              />
              <KeyValue
                label="左 / 右畸变系数"
                value={`${result.optics.left_d.join(" / ")}；${result.optics.right_d.join(" / ")}`}
              />
              <KeyValue
                label="左到右平移（m）"
                value={result.optics.left_to_right.position_m
                  .map((v) => v.toFixed(6))
                  .join(" / ")}
              />
              <KeyValue
                label="左到右四元数（xyzw）"
                value={result.optics.left_to_right.orientation_xyzw
                  .map((v) => v.toFixed(6))
                  .join(" / ")}
              />
              {result.camera_pose && (
                <>
                  <KeyValue
                    label={
                      rig === "external"
                        ? "左相机在底座中的位置（m）"
                        : "左相机在 TCP 中的位置（m）"
                    }
                    value={result.camera_pose.position_m
                      .map((v) => v.toFixed(6))
                      .join(" / ")}
                  />
                  <KeyValue
                    label="相机外参四元数（xyzw）"
                    value={result.camera_pose.orientation_xyzw
                      .map((v) => v.toFixed(6))
                      .join(" / ")}
                  />
                </>
              )}
            </CalibrationValues>
          </Disclosure>
        </section>
      )}
      <div className="card-actions">
        <Button disabled={busy} onClick={() => act("apply")}>
          保存并应用标定
        </Button>
        <Button
          disabled={pending}
          variant="outline"
          onClick={() => act("reset")}
        >
          {session?.busy ? "取消本轮标定" : "重新标定"}
        </Button>
        <Button
          disabled={busy}
          variant="outline"
          onClick={async () => {
            setPending(true);
            setError(undefined);
            setDepth(undefined);
            try {
              const response = await fetch(
                `/api/camera/stereo/depth?rig=${rig}`,
              );
              const value = await response.json();
              if (!response.ok) throw Error(value.original_error);
              setDepth(value);
            } catch (e) {
              setError(String(e));
            } finally {
              setPending(false);
            }
          }}
        >
          检查深度
        </Button>
      </div>
      <p role="status">
        {pending ? "正在处理请求…" : session?.message || "尚未开始"}
      </p>
      {(error || connectionError || session?.original_error) && (
        <p className="error" role="alert">
          {error || connectionError || session?.original_error}
        </p>
      )}
      {depth && (
        <section className="camera-binding">
          <KeyValue
            label="有效深度像素"
            value={`${depth.valid_pixels} / ${depth.total_pixels}`}
          />
          <KeyValue
            label="深度范围"
            value={`${Number(depth.min_depth_m).toFixed(3)} ～ ${Number(depth.max_depth_m).toFixed(3)} m`}
          />
          <p className="status">
            查看深度图及点云：刷新相机列表，选择该组双目深度来源并启用；复用现有深度展示与三维定位。
          </p>
        </section>
      )}
    </Card>
  );
}
