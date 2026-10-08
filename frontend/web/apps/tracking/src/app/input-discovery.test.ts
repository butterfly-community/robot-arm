import { describe, expect, it } from "vitest";
import { inputDiscoveryNotices } from "./input-discovery";

const drivers = ["sdl3-gamepad", "nolo-cv1-hid"].map((driver_id) => ({
  driver_id,
  original_error: null,
}));
describe("input discovery messages", () => {
  it("does not mistake a disconnected or waiting service for missing hardware", () => {
    expect(
      inputDiscoveryNotices("disconnected", { drivers, sources: [] })[0]
        .message,
    ).toContain("无法确认当前设备状态");
    expect(inputDiscoveryNotices("connecting", undefined)[0].message).toContain(
      "正在连接",
    );
    expect(inputDiscoveryNotices("connected", undefined)[0].message).toContain(
      "等待输入节点",
    );
    expect(
      inputDiscoveryNotices("connected", { drivers: [] })[0].message,
    ).toContain("尚未上报");
  });
  it("reports no SDL gamepad without asserting a permissions failure", () => {
    const notices = inputDiscoveryNotices("connected", {
      drivers,
      sources: [],
    });
    expect(notices[0].message).toContain("SDL 未发现可用手柄");
    expect(notices[0].error).toBe(false);
    expect(notices[0].message).not.toContain("权限不足");
  });
  it("does not let simulation or NOLO hide missing SDL hardware", () => {
    const sources = [
      { source_id: "simulation", driver_id: "simulation", active: true },
      { driver_id: "nolo-cv1-hid", active: true },
    ];
    expect(
      inputDiscoveryNotices("connected", { drivers, sources }),
    ).toHaveLength(1);
    expect(inputDiscoveryNotices("connected", { drivers, sources })[0].id).toBe(
      "sdl3-gamepad",
    );
  });
  it("shows original driver errors instead of the generic empty-device hint", () => {
    const original_error =
      "SDL3 gamepad 采集线程已停止：启用陀螺仪失败（DualSense）：Operation not supported";
    const notices = inputDiscoveryNotices("connected", {
      drivers: [{ ...drivers[0], original_error }, drivers[1]],
      sources: [],
    });
    expect(notices[0].error).toBe(true);
    expect(notices[0].message).toContain(original_error);
    expect(notices[0].message).not.toContain("SDL 未发现");
  });
  it("clears the empty hint when a real device appears and shows device errors", () => {
    const sources = [
      {
        source_id: "pad",
        driver_id: "sdl3-gamepad",
        active: true,
        display_name: "手柄",
        original_error: "sensor failed",
      },
    ];
    const notices = inputDiscoveryNotices("connected", { drivers, sources });
    expect(notices.some((item) => item.id === "sdl3-gamepad")).toBe(false);
    expect(notices.find((item) => item.id === "source:pad")?.message).toBe(
      "手柄：sensor failed",
    );
  });
});
