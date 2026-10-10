"use client";

import { RGB888 } from "@thi.ng/pixel";
import type { CameraRole } from "@robot/contracts";
import { cameraTitles } from "./camera-bindings";
import { Button } from "@robot/ui";
import {
  type CSSProperties,
  type PointerEvent as ReactPointerEvent,
  useEffect,
  useRef,
  useState,
  useSyncExternalStore,
} from "react";

type RawFrame = {
  width: number;
  height: number;
  stride: number;
  format: string;
  bytes: Uint8Array;
};

export function rawFrameToRgba(
  frame: RawFrame,
): Uint8ClampedArray<ArrayBuffer> {
  if (frame.format !== "rgb8")
    throw new Error(`不支持的彩色视频格式 ${frame.format}，相机应发布 rgb8`);
  const bytesPerPixel = 3;
  if (
    frame.width <= 0 ||
    frame.height <= 0 ||
    frame.stride < frame.width * bytesPerPixel ||
    frame.bytes.length !== frame.stride * frame.height
  ) {
    throw new Error("彩色视频帧尺寸、步长或载荷长度无效");
  }
  const pixels = new Uint32Array(frame.width * frame.height);
  for (let y = 0; y < frame.height; y += 1) {
    for (let x = 0; x < frame.width; x += 1) {
      const source = y * frame.stride + x * bytesPerPixel;
      let packed = 0;
      for (let channel = 0; channel < bytesPerPixel; channel += 1) {
        packed = (packed << 8) | frame.bytes[source + channel];
      }
      pixels[y * frame.width + x] = RGB888.toABGR(packed);
    }
  }
  // @thi.ng/pixel 的统一中间格式就是 Canvas 原生 ABGR32；在浏览器的
  // Uint8ClampedArray 视图中对应 RGBA 字节，不再自行维护通道转换分支。
  return new Uint8ClampedArray(pixels.buffer);
}

type FrameMetadata = {
  width: number;
  height: number;
  stride_bytes: number;
  pixel_format: string;
};

function frameMetadata(value: string): FrameMetadata {
  const metadata = JSON.parse(value) as Partial<FrameMetadata>;
  if (
    !Number.isSafeInteger(metadata.width) ||
    !Number.isSafeInteger(metadata.height) ||
    !Number.isSafeInteger(metadata.stride_bytes) ||
    !metadata.pixel_format
  ) {
    throw new Error("相机视频元数据无效");
  }
  return metadata as FrameMetadata;
}

export function CameraVideo({
  streaming,
  role = "depth",
}: {
  streaming: boolean;
  role?: CameraRole;
}) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const [error, setError] = useState<string>();

  useEffect(() => {
    if (!streaming) return;
    let stopped = false;
    let socket: WebSocket | undefined;
    let retry: ReturnType<typeof setTimeout> | undefined;
    let metadata: FrameMetadata | undefined;
    const connect = () => {
      metadata = undefined;
      const scheme = location.protocol === "https:" ? "wss" : "ws";
      socket = new WebSocket(
        `${scheme}://${location.host}/ws/camera-video?role=${role}`,
      );
      socket.binaryType = "arraybuffer";
      socket.onmessage = (event) => {
        try {
          if (typeof event.data === "string") {
            if (JSON.parse(event.data).signal === false) {
              canvas.current
                ?.getContext("2d")
                ?.clearRect(0, 0, canvas.current.width, canvas.current.height);
              metadata = undefined;
              setError("相机信号中断或未绑定");
              return;
            }
            metadata = frameMetadata(event.data);
            return;
          }
          if (!metadata || !(event.data instanceof ArrayBuffer)) {
            throw new Error("相机视频帧与元数据未配对");
          }
          const frame = {
            width: metadata.width,
            height: metadata.height,
            stride: metadata.stride_bytes,
            format: metadata.pixel_format,
            bytes: new Uint8Array(event.data),
          };
          metadata = undefined;
          const target = canvas.current;
          if (!target) return;
          target.width = frame.width;
          target.height = frame.height;
          const context = target.getContext("2d");
          if (!context) throw new Error("浏览器不支持 Canvas 2D");
          context.putImageData(
            new ImageData(rawFrameToRgba(frame), frame.width, frame.height),
            0,
            0,
          );
          setError(undefined);
        } catch (reason) {
          setError(reason instanceof Error ? reason.message : String(reason));
        }
      };
      socket.onclose = () => {
        if (!stopped) {
          canvas.current
            ?.getContext("2d")
            ?.clearRect(0, 0, canvas.current.width, canvas.current.height);
          setError("相机视频连接中断，正在重连");
          retry = setTimeout(connect, 1000);
        }
      };
      socket.onerror = () => setError("相机视频连接中断，正在重连");
    };
    connect();
    return () => {
      stopped = true;
      if (retry) clearTimeout(retry);
      socket?.close();
    };
  }, [streaming, role]);

  if (!streaming) return <div className="visual-empty">等待相机开始采集</div>;
  return (
    <div className="camera-video">
      <canvas ref={canvas} aria-label="相机原始彩色视频" />
      {error && <span className="camera-video-error">{error}</span>}
    </div>
  );
}

type FloatingCameraVideoProps = {
  role?: CameraRole;
  hasSignal?: boolean;
  streaming: boolean;
  profile: string;
  rates: string;
};

type WindowPosition = {
  left: number;
  top: number;
};

const POSITION_STORAGE_KEY = "robot-arm:perception:color-video-position";
const COLLAPSED_STORAGE_KEY = "robot-arm:perception:color-video-collapsed";
const subscribeBrowser = () => () => undefined;
const browserSnapshot = () => true;
const serverSnapshot = () => false;

function storedPosition(key: string): WindowPosition | undefined {
  if (typeof window === "undefined") return undefined;
  try {
    const value = JSON.parse(
      localStorage.getItem(key) ?? "null",
    ) as Partial<WindowPosition> | null;
    if (
      Number.isFinite(value?.left) &&
      Number.isFinite(value?.top) &&
      value!.left! >= 0 &&
      value!.top! >= 0 &&
      value!.left! < window.innerWidth - 48 &&
      value!.top! < window.innerHeight - 48
    ) {
      return { left: value!.left!, top: value!.top! };
    }
  } catch {
    // Ignore stale browser state and use the default corner.
  }
  return undefined;
}

function storedCollapsed(key: string): boolean {
  if (typeof window === "undefined") return false;
  return localStorage.getItem(key) === "true";
}

export function FloatingCameraVideo({
  role = "depth",
  streaming,
  hasSignal = streaming,
  profile,
  rates,
}: FloatingCameraVideoProps) {
  const browserReady = useSyncExternalStore(
    subscribeBrowser,
    browserSnapshot,
    serverSnapshot,
  );
  const [seen, setSeen] = useState(false);
  if (hasSignal && !seen) setSeen(true);
  if (!browserReady || (!hasSignal && !seen)) return null;
  return (
    <FloatingCameraVideoWindow
      role={role}
      hasSignal={hasSignal}
      streaming={streaming}
      profile={profile}
      rates={rates}
    />
  );
}

function FloatingCameraVideoWindow({
  role = "depth",
  streaming,
  hasSignal = streaming,
  profile,
  rates,
}: FloatingCameraVideoProps) {
  const positionKey = `${POSITION_STORAGE_KEY}:${role}`;
  const collapsedKey = `${COLLAPSED_STORAGE_KEY}:${role}`;
  const panel = useRef<HTMLElement>(null);
  const dragOffset = useRef<WindowPosition | undefined>(undefined);
  const [position, setPosition] = useState<WindowPosition | undefined>(() =>
    storedPosition(positionKey),
  );
  const positionRef = useRef<WindowPosition | undefined>(position);
  const [collapsed, setCollapsed] = useState(() =>
    storedCollapsed(collapsedKey),
  );

  const startDrag = (event: ReactPointerEvent<HTMLElement>) => {
    if (event.button !== 0 || !panel.current) return;
    const bounds = panel.current.getBoundingClientRect();
    dragOffset.current = {
      left: event.clientX - bounds.left,
      top: event.clientY - bounds.top,
    };
    event.currentTarget.setPointerCapture(event.pointerId);
  };

  const drag = (event: ReactPointerEvent<HTMLElement>) => {
    if (!dragOffset.current || !panel.current) return;
    const bounds = panel.current.getBoundingClientRect();
    const nextPosition = {
      left: Math.min(
        Math.max(0, event.clientX - dragOffset.current.left),
        Math.max(0, window.innerWidth - bounds.width),
      ),
      top: Math.min(
        Math.max(0, event.clientY - dragOffset.current.top),
        Math.max(0, window.innerHeight - bounds.height),
      ),
    };
    positionRef.current = nextPosition;
    setPosition(nextPosition);
  };

  const stopDrag = (event: ReactPointerEvent<HTMLElement>) => {
    dragOffset.current = undefined;
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
    if (positionRef.current) {
      localStorage.setItem(positionKey, JSON.stringify(positionRef.current));
    }
  };

  const style = position
    ? ({
        left: position.left,
        top: position.top,
        right: "auto",
        bottom: "auto",
      } satisfies CSSProperties)
    : ({
        top: 80 + ["depth", "external", "wrist"].indexOf(role) * 64,
        bottom: "auto",
      } satisfies CSSProperties);

  return (
    <aside
      ref={panel}
      className={`floating-camera-monitor${collapsed ? " is-collapsed" : ""}`}
      style={style}
      aria-label={`${cameraTitles[role]}浮动窗口`}
    >
      <header
        className="floating-camera-header"
        onPointerDown={startDrag}
        onPointerMove={drag}
        onPointerUp={stopDrag}
        onPointerCancel={stopDrag}
      >
        <div>
          <strong>{cameraTitles[role]}</strong>
          <span>
            {!hasSignal ? "信号中断" : collapsed ? "预览已收起" : "采集中"}
          </span>
        </div>
        <Button
          type="button"
          variant="outline"
          className="floating-camera-toggle"
          aria-expanded={!collapsed}
          title={
            collapsed
              ? "展开并连接视频预览"
              : "收起并断开视频预览，不停止相机采集"
          }
          onPointerDown={(event) => event.stopPropagation()}
          onClick={() => {
            const next = !collapsed;
            setCollapsed(next);
            localStorage.setItem(collapsedKey, String(next));
          }}
        >
          {collapsed ? "展开" : "收起"}
        </Button>
      </header>
      {!collapsed && (
        <div className="floating-camera-content">
          <CameraVideo streaming={streaming && hasSignal} role={role} />
          <div className="floating-camera-metadata">
            <span title={profile}>{profile}</span>
            <span title={rates}>{rates}</span>
          </div>
        </div>
      )}
    </aside>
  );
}
