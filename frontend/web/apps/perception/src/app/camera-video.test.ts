import { describe, expect, it } from "vitest";

import { rawFrameToRgba } from "./camera-video";

describe("rawFrameToRgba", () => {
  it("converts padded BGR rows with an opaque alpha channel", () => {
    expect(
      Array.from(
        rawFrameToRgba({
          width: 1,
          height: 2,
          stride: 4,
          format: "bgr8",
          bytes: new Uint8Array([3, 2, 1, 99, 30, 20, 10, 88]),
        }),
      ),
    ).toEqual([1, 2, 3, 255, 10, 20, 30, 255]);
  });

  it("expands monochrome frames", () => {
    expect(
      Array.from(
        rawFrameToRgba({
          width: 2,
          height: 1,
          stride: 2,
          format: "y8",
          bytes: new Uint8Array([7, 9]),
        }),
      ),
    ).toEqual([7, 7, 7, 255, 9, 9, 9, 255]);
  });

  it("uses declared channel order for four-channel frames", () => {
    expect(
      Array.from(
        rawFrameToRgba({
          width: 2,
          height: 1,
          stride: 8,
          format: "bgra8",
          bytes: new Uint8Array([3, 2, 1, 4, 30, 20, 10, 40]),
        }),
      ),
    ).toEqual([1, 2, 3, 4, 10, 20, 30, 40]);
  });

  it("rejects formats outside the camera frame contract", () => {
    expect(() =>
      rawFrameToRgba({
        width: 1,
        height: 1,
        stride: 2,
        format: "yuyv",
        bytes: new Uint8Array(2),
      }),
    ).toThrow("不支持的彩色视频格式 yuyv");
  });
});
