"use client";

import {
  inputActionCatalog,
  schemaVersion,
  type InputSimulationItem,
} from "@robot/contracts";
import { post, requestId } from "@robot/gateway-client";
import { Button, Card, HelpDot } from "@robot/ui";
import { useCallback, useEffect, useRef, useState } from "react";

const directions = [
  ["forward", "前进", "↑", "move_forward_back", 1],
  ["left", "左移", "←", "move_left_right", 1],
  ["right", "右移", "→", "move_left_right", -1],
  ["back", "后退", "↓", "move_forward_back", -1],
  ["up", "上移", "↑", "move_up_down", 1],
  ["down", "下移", "↓", "move_up_down", -1],
  ["pitch-up", "俯仰抬起", "↶", "tool_pitch", 1],
  ["pitch-down", "俯仰往下", "↷", "tool_pitch", -1],
  ["yaw-left", "向左转", "↶", "tool_yaw", 1],
  ["yaw-right", "向右转", "↷", "tool_yaw", -1],
  ["roll-left", "轴向逆时针", "↶", "tool_roll", 1],
  ["roll-right", "轴向顺时针", "↷", "tool_roll", -1],
] as const satisfies readonly (readonly [
  string,
  string,
  string,
  InputSimulationItem,
  number,
])[];

export function MouseControl({
  onError,
}: {
  onError: (message: string | undefined) => void;
}) {
  const [active, setActive] = useState<string>();
  const held = useRef<symbol | undefined>(undefined);
  const started = useRef(false);
  const queue = useRef(Promise.resolve());

  const release = useCallback(() => {
    held.current = undefined;
    setActive(undefined);
    queue.current = queue.current
      .then(async () => {
        if (!started.current) return;
        await post(
          "/api/tracking/simulation",
          {
            schema_version: schemaVersion,
            request_id: requestId(),
            enabled: false,
            item: null,
          },
          { keepalive: true },
        );
        started.current = false;
      })
      .catch((reason) => onError(String(reason)));
  }, [onError]);

  useEffect(() => {
    const stop = release;
    const visibility = () => {
      if (document.hidden) stop();
    };
    window.addEventListener("blur", stop);
    window.addEventListener("pagehide", stop);
    document.addEventListener("visibilitychange", visibility);
    return () => {
      stop();
      window.removeEventListener("blur", stop);
      window.removeEventListener("pagehide", stop);
      document.removeEventListener("visibilitychange", visibility);
    };
  }, [release]);

  function press(direction: (typeof directions)[number]) {
    if (held.current) return;
    const [key, , , item, value] = direction;
    const pressId = Symbol(key);
    held.current = pressId;
    setActive(key);
    onError(undefined);
    queue.current = queue.current
      .then(async () => {
        if (held.current !== pressId) return;
        await post("/api/motion/mode", {
          schema_version: schemaVersion,
          request_id: requestId(),
          mode: "relative",
        });
        if (held.current !== pressId) return;
        await post("/api/tracking/simulation", {
          schema_version: schemaVersion,
          request_id: requestId(),
          enabled: true,
          item,
          value,
        });
        started.current = true;
      })
      .catch((reason) => {
        held.current = undefined;
        setActive(undefined);
        onError(String(reason));
      });
  }

  function button(direction: (typeof directions)[number]) {
    const [key, label, icon, item] = direction;
    const action = inputActionCatalog.find((action) => action.key === item)!;
    return (
      <Button
        key={key}
        variant="outline"
        className={`mouse-direction mouse-${key}`}
        aria-label={label}
        aria-pressed={active === key}
        title={`${action.motion}；${action.invariant}。${action.semantics}。按住运动，松开停止。`}
        onPointerDown={(event) => {
          if (event.button !== 0) return;
          event.preventDefault();
          event.currentTarget.setPointerCapture(event.pointerId);
          press(direction);
        }}
        onPointerUp={release}
        onPointerCancel={release}
        onLostPointerCapture={release}
        onKeyDown={(event) => {
          if ((event.key === " " || event.key === "Enter") && !event.repeat) {
            event.preventDefault();
            press(direction);
          }
        }}
        onKeyUp={(event) => {
          if (event.key === " " || event.key === "Enter") {
            event.preventDefault();
            release();
          }
        }}
        onBlur={release}
      >
        <span aria-hidden="true">{icon}</span>
        <span>{label}</span>
      </Button>
    );
  }

  return (
    <Card className="span-12" title="鼠标方向控制" eyebrow="Mouse control">
      <p className="status">
        按住运动，松开停止；从当前姿态接管，不回默认位。
        <HelpDot text="平移保持工具朝向；姿态旋转保持 TCP 位置。使用空间页配置的线速度和角速度。临时接管输入，结束后恢复已有绑定，不保存鼠标输入为设备配置。轴向旋转从机械臂后部朝尖端观察。" />
      </p>
      <div className="mouse-control-layout">
        <div className="mouse-pad" role="group" aria-label="水平平移方向盘">
          {directions.slice(0, 4).map(button)}
          <Button variant="outline" className="mouse-stop" onClick={release}>
            停止
          </Button>
        </div>
        <div className="mouse-vertical" role="group" aria-label="上下移动">
          {directions.slice(4, 6).map(button)}
        </div>
        <div className="mouse-orientation" role="group" aria-label="自身姿态">
          {directions.slice(6).map(button)}
        </div>
      </div>
    </Card>
  );
}
