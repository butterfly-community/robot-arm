"use client";

import {
  BGR888,
  GRAY8,
  Lane,
  RGB888,
  defIntFormat,
  type IntFormat,
} from "@thi.ng/pixel";
import { useEffect, useRef, useState } from "react";

type RawFrame = {
  width: number;
  height: number;
  stride: number;
  format: string;
  bytes: Uint8Array;
};

const RGBA8888 = defIntFormat({
  type: "u32",
  size: 32,
  alpha: 8,
  channels: [
    { size: 8, lane: Lane.RED },
    { size: 8, lane: Lane.GREEN },
    { size: 8, lane: Lane.BLUE },
    { size: 8, lane: Lane.ALPHA },
  ],
});

const BGRA8888 = defIntFormat({
  type: "u32",
  size: 32,
  alpha: 8,
  channels: [
    { size: 8, lane: Lane.BLUE },
    { size: 8, lane: Lane.GREEN },
    { size: 8, lane: Lane.RED },
    { size: 8, lane: Lane.ALPHA },
  ],
});

const PIXEL_FORMATS: Record<
  string,
  { bytesPerPixel: number; format: IntFormat }
> = {
  rgb8: { bytesPerPixel: 3, format: RGB888 },
  bgr8: { bytesPerPixel: 3, format: BGR888 },
  rgba8: { bytesPerPixel: 4, format: RGBA8888 },
  bgra8: { bytesPerPixel: 4, format: BGRA8888 },
  y8: { bytesPerPixel: 1, format: GRAY8 },
};

export function rawFrameToRgba(
  frame: RawFrame,
): Uint8ClampedArray<ArrayBuffer> {
  const sourceFormat = PIXEL_FORMATS[frame.format];
  if (!sourceFormat) throw new Error(`不支持的彩色视频格式 ${frame.format}`);
  const { bytesPerPixel, format } = sourceFormat;
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
      pixels[y * frame.width + x] = format.toABGR(packed);
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

export function CameraVideo({ streaming }: { streaming: boolean }) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const [error, setError] = useState<string>();

  useEffect(() => {
    if (!streaming) return;
    let stopped = false;
    let socket: WebSocket | undefined;
    let retry: ReturnType<typeof setTimeout> | undefined;
    let metadata: FrameMetadata | undefined;
    const connect = () => {
      const scheme = location.protocol === "https:" ? "wss" : "ws";
      socket = new WebSocket(`${scheme}://${location.host}/ws/camera-video`);
      socket.binaryType = "arraybuffer";
      socket.onmessage = (event) => {
        try {
          if (typeof event.data === "string") {
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
        if (!stopped) retry = setTimeout(connect, 1000);
      };
      socket.onerror = () => setError("相机视频连接中断，正在重连");
    };
    connect();
    return () => {
      stopped = true;
      if (retry) clearTimeout(retry);
      socket?.close();
    };
  }, [streaming]);

  if (!streaming) return <div className="visual-empty">等待相机开始采集</div>;
  return (
    <div className="camera-video">
      <canvas ref={canvas} aria-label="相机原始彩色视频" />
      {error && <span className="camera-video-error">{error}</span>}
    </div>
  );
}
