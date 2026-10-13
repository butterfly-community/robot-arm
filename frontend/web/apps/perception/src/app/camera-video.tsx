"use client";

import type JMuxer from "jmuxer";
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

type FrameMetadata = {
  width: number;
  height: number;
  codec: "h264";
  stream_id: number;
  sequence: number;
  keyframe: boolean;
  duration_ms: number;
};

type VideoPacket = Parameters<JMuxer["feed"]>[0];

export function videoChunk(packets: readonly VideoPacket[]): VideoPacket {
  const video = new Uint8Array(
    packets.reduce((n, p) => n + p.video.byteLength, 0),
  );
  let offset = 0;
  let duration = 0;
  for (const packet of packets) {
    video.set(packet.video, offset);
    offset += packet.video.byteLength;
    duration += packet.duration;
  }
  // JMuxer parses the access units and apportions the chunk's duration across
  // its frames. Preserve every byte and total capture time; don't hand-write
  // a cadence estimator, duplicate frames or resample camera pixels.
  return { video, duration, isLastVideoFrameComplete: true };
}

export function frameMetadata(value: string): FrameMetadata {
  const metadata = JSON.parse(value) as Partial<FrameMetadata>;
  if (
    !Number.isSafeInteger(metadata.width) ||
    !Number.isSafeInteger(metadata.height) ||
    metadata.width! <= 0 ||
    metadata.height! <= 0 ||
    metadata.codec !== "h264" ||
    !Number.isSafeInteger(metadata.stream_id) ||
    !Number.isSafeInteger(metadata.sequence) ||
    typeof metadata.keyframe !== "boolean" ||
    !Number.isFinite(metadata.duration_ms) ||
    metadata.duration_ms! <= 0
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
  const video = useRef<HTMLVideoElement>(null);
  const [error, setError] = useState<string>();
  const [fps, setFps] = useState<number>();

  useEffect(() => {
    if (!streaming) return;
    let stopped = false;
    let socket: WebSocket | undefined;
    let retry: ReturnType<typeof setTimeout> | undefined;
    let metadata: FrameMetadata | undefined;
    let muxer: JMuxer | undefined;
    let muxerReady = false;
    let pending: VideoPacket[] = [];
    let streamId: number | undefined;
    let durationRemainder = 0;
    let callback: number | undefined;
    const target = video.current;
    if (!target) return;
    let lastPaint: { time: number; frames: number } | undefined;
    const painted = (now: number, frame: VideoFrameCallbackMetadata) => {
      if (stopped) return;
      if (!lastPaint) lastPaint = { time: now, frames: frame.presentedFrames };
      if (now - lastPaint.time >= 1000) {
        setFps(
          ((frame.presentedFrames - lastPaint.frames) * 1000) /
            (now - lastPaint.time),
        );
        lastPaint = { time: now, frames: frame.presentedFrames };
      }
      callback = target.requestVideoFrameCallback(painted);
    };
    if (target.requestVideoFrameCallback)
      callback = target.requestVideoFrameCallback(painted);
    const clear = () => {
      muxer?.destroy();
      muxer = undefined;
      muxerReady = false;
      pending = [];
      streamId = undefined;
      metadata = undefined;
      durationRemainder = 0;
      lastPaint = undefined;
      setFps(undefined);
      target.removeAttribute("src");
      target.load();
    };
    const flush = () => {
      if (!muxerReady || !muxer || !pending.length) return;
      try {
        const chunk = videoChunk(pending);
        pending = [];
        muxer.feed(chunk);
      } catch (reason) {
        clear();
        setError(reason instanceof Error ? reason.message : String(reason));
      }
    };
    const feedTimer = setInterval(flush, 200);
    const connect = (Muxer: typeof JMuxer) => {
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
              clear();
              setError("相机信号中断或未绑定");
              return;
            }
            metadata = frameMetadata(event.data);
            return;
          }
          if (!metadata || !(event.data instanceof ArrayBuffer)) {
            throw new Error("相机视频帧与元数据未配对");
          }
          const frame = metadata;
          metadata = undefined;
          if (streamId !== frame.stream_id) {
            clear();
            if (!frame.keyframe) throw Error("H.264 视频缺少起始关键帧");
            streamId = frame.stream_id;
            const created = new Muxer({
              node: target,
              mode: "video",
              videoCodec: "H264",
              // Feed complete short chunks, then append them immediately.
              // The user permits 1–2 s buffering; avoid the library's default
              // 500 ms live-edge seek discarding otherwise valid frames.
              flushingTime: 0,
              maxDelay: 2000,
              clearBuffer: true,
              live: true,
              onReady: () => {
                if (stopped || muxer !== created) return;
                muxerReady = true;
                flush();
              },
              onUnsupportedCodec: () => {
                if (!stopped) setError("当前浏览器不支持此 H.264 视频解码");
              },
              onError: (reason: unknown) => {
                if (!stopped) setError(`H.264 播放失败：${String(reason)}`);
              },
            });
            muxer = created;
            void target.play().catch((reason: unknown) => {
              if (
                !stopped &&
                reason instanceof Error &&
                reason.name !== "AbortError"
              )
                setError(reason.message);
            });
          }
          const duration = frame.duration_ms + durationRemainder;
          const rounded = Math.round(duration);
          durationRemainder = duration - rounded;
          const packet = {
            video: new Uint8Array(event.data),
            duration: rounded,
            isLastVideoFrameComplete: true,
          };
          // MSE sourceopen is asynchronous. Feeding before onReady discards
          // JMuxer's initialization segment and leaves the video unplayable.
          pending.push(packet);
        } catch (reason) {
          clear();
          setError(reason instanceof Error ? reason.message : String(reason));
        }
      };
      socket.onclose = () => {
        if (!stopped) {
          clear();
          setError("相机视频连接中断，正在重连");
          retry = setTimeout(() => connect(Muxer), 1000);
        }
      };
      socket.onerror = () => {
        if (!stopped) setError("相机视频连接中断，正在重连");
      };
    };
    void import("jmuxer")
      .then(({ default: Muxer }) => {
        if (!stopped) connect(Muxer);
      })
      .catch((reason: unknown) => {
        if (!stopped) setError(`H.264 播放器加载失败：${String(reason)}`);
      });
    return () => {
      stopped = true;
      clearInterval(feedTimer);
      if (retry) clearTimeout(retry);
      socket?.close();
      if (callback !== undefined) target.cancelVideoFrameCallback(callback);
      clear();
    };
  }, [streaming, role]);

  if (!streaming) return <div className="visual-empty">等待相机开始采集</div>;
  return (
    <div className="camera-video">
      <video
        ref={video}
        aria-label="相机彩色视频"
        autoPlay
        muted
        playsInline
        onPlaying={() => setError(undefined)}
      />
      <span className="camera-video-rate">
        H.264 · {fps === undefined ? "等待播放" : `${fps.toFixed(1)} FPS 实测`}
      </span>
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
        top:
          80 +
          [
            "depth",
            "external",
            "external_right",
            "wrist",
            "wrist_right",
          ].indexOf(role) *
            64,
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
          <strong title={cameraTitles[role]}>{cameraTitles[role]}</strong>
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
