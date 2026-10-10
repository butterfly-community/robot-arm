"use client";
import { useState } from "react";
import type { CameraCaptureState, CameraRole } from "@robot/contracts";
import { Button, Field, KeyValue } from "@robot/ui";

export const cameraTitles: Record<CameraRole, string> = {
  depth: "深度相机彩色画面",
  external: "外部摄像头",
  wrist: "腕部摄像头",
};

export function CameraBindings({
  camera,
  onRequest,
}: {
  camera?: CameraCaptureState;
  onRequest: (body: Record<string, unknown>) => Promise<boolean>;
}) {
  return (
    <div className="camera-bindings">
      <p className="status">
        外部摄像头用于观察场景，腕部摄像头随夹爪移动。绑定表示图像用途，不代表已经完成几何标定。相机收到图像后显示浮动预览。
      </p>
      <p className="status">
        共享 USB
        带宽不足时，可先关闭一路采集、再开启另一路，轮流采集以缓解带宽占用。关闭采集会释放设备，保留绑定和分辨率；再次开启后使用新图像，不复用关闭前的画面。
      </p>
      {(["external", "wrist"] as const).map((role) => (
        <Binding key={role} role={role} camera={camera} onRequest={onRequest} />
      ))}
    </div>
  );
}

function Binding({
  role,
  camera,
  onRequest,
}: {
  role: "external" | "wrist";
  camera?: CameraCaptureState;
  onRequest: (body: Record<string, unknown>) => Promise<boolean>;
}) {
  const binding = camera?.bindings?.find((b) => b.role === role);
  const [draft, setDraft] = useState<{ source: string; profile: string }>();
  const [pending, setPending] = useState(false);
  const sourceId = draft?.source ?? binding?.source_id ?? "";
  const source = camera?.available_sources.find(
    (s) => s.source_id === sourceId,
  );
  const profiles = source?.profiles.filter((p) => p.stream === "color") ?? [];
  const profile = draft?.profile ?? binding?.color_profile_key ?? "";
  const submit = async (action: string) => {
    setPending(true);
    try {
      if (
        await onRequest({
          action,
          role,
          source_id: sourceId || null,
          color_profile_key: profile || null,
        })
      )
        setDraft(undefined);
    } finally {
      setPending(false);
    }
  };
  return (
    <section
      className="camera-binding"
      aria-label={`${cameraTitles[role]}绑定`}
    >
      <h3>{cameraTitles[role]}</h3>
      <Field
        label={`${cameraTitles[role]}来源`}
        hint="绑定按稳定设备标识保存，设备编号变化不影响配置；无序列号设备按物理端口识别。"
      >
        <select
          aria-label={`${cameraTitles[role]}来源`}
          value={sourceId}
          disabled={pending}
          onChange={(e) => {
            const source = camera?.available_sources.find(
              (s) => s.source_id === e.target.value,
            );
            setDraft({
              source: e.target.value,
              profile:
                source?.profiles.find(
                  (p) => p.stream === "color" && p.available,
                )?.key ?? "",
            });
          }}
        >
          <option value="">不绑定</option>
          {camera?.available_sources.map((s) => (
            <option
              key={s.source_id}
              value={s.source_id}
              disabled={!s.available}
            >
              {s.display_name}
              {s.available ? "" : " · 未连接（绑定保留）"}
            </option>
          ))}
        </select>
      </Field>
      <Field label={`${cameraTitles[role]}流配置`}>
        <select
          aria-label={`${cameraTitles[role]}流配置`}
          value={profile}
          disabled={pending}
          onChange={(e) =>
            setDraft({ source: sourceId, profile: e.target.value })
          }
        >
          <option value="">选择分辨率和帧率</option>
          {profiles.map((p) => (
            <option key={p.key} value={p.key} disabled={!p.available}>
              {p.width} × {p.height} · {p.frames_per_second} FPS ·{" "}
              {p.pixel_format}
              {p.available ? "" : ` · ${p.unavailable_reason ?? "不可用"}`}
            </option>
          ))}
        </select>
      </Field>
      <div className="card-actions">
        <Button
          variant="outline"
          disabled={pending}
          onClick={() => submit(sourceId ? "apply" : "unselect")}
        >
          {pending ? "正在保存…" : `保存${cameraTitles[role]}绑定`}
        </Button>
        {binding && (
          <Button
            variant="outline"
            disabled={pending}
            onClick={() =>
              submit(binding.enabled !== false ? "disconnect" : "connect")
            }
          >
            {binding.enabled !== false
              ? `关闭${cameraTitles[role]}采集`
              : `开启${cameraTitles[role]}采集`}
          </Button>
        )}
        {binding && (
          <Button
            variant="outline"
            disabled={pending}
            onClick={() => submit("unselect")}
          >
            解除绑定
          </Button>
        )}
        {draft && (
          <Button
            variant="outline"
            disabled={pending}
            onClick={() => setDraft(undefined)}
          >
            恢复已保存绑定
          </Button>
        )}
      </div>
      <KeyValue
        label="采集状态"
        value={
          binding?.original_error ??
          (binding?.has_signal
            ? "正在采集"
            : binding?.enabled === false
              ? "采集已关闭，绑定与分辨率保留"
              : binding
                ? "已绑定，等待图像信号"
                : "未绑定")
        }
      />
      {binding?.frame && (
        <KeyValue
          label="实际图像"
          value={`${binding.frame.width} × ${binding.frame.height} · ${binding.frame.encoding}`}
        />
      )}
    </section>
  );
}
