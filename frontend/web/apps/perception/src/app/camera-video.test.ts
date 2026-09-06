import { describe, expect, it } from "vitest";

import { rawFrameToRgba } from "./camera-video";

describe("rawFrameToRgba", () => {
  it.each([
    [1280, 720],
    [1920, 1080],
  ])("uses the reported %d×%d frame size", (width, height) => {
    const rgba = rawFrameToRgba({
      width,
      height,
      stride: width * 3,
      format: "rgb8",
      bytes: new Uint8Array(width * height * 3),
    });
    expect(rgba).toHaveLength(width * height * 4);
  });

  it("reads padded RGB rows without swapping channels", () => {
    expect(
      Array.from(
        rawFrameToRgba({
          width: 1,
          height: 2,
          stride: 4,
          format: "rgb8",
          bytes: new Uint8Array([1, 2, 3, 99, 10, 20, 30, 88]),
        }),
      ),
    ).toEqual([1, 2, 3, 255, 10, 20, 30, 255]);
  });

  it("preserves red, green and blue primaries", () => {
    expect(
      Array.from(
        rawFrameToRgba({
          width: 3,
          height: 1,
          stride: 9,
          format: "rgb8",
          bytes: new Uint8Array([255, 0, 0, 0, 255, 0, 0, 0, 255]),
        }),
      ),
    ).toEqual([255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255]);
  });

  it.each(["bgr8", "rgba8", "bgra8", "y8", "yuyv"])(
    "rejects native %s outside the published RGB contract",
    (format) => {
      expect(() =>
        rawFrameToRgba({
          width: 1,
          height: 1,
          stride: 2,
          format,
          bytes: new Uint8Array(2),
        }),
      ).toThrow(`不支持的彩色视频格式 ${format}`);
    },
  );
});
