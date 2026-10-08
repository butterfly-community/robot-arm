import type { ConnectionState } from "@robot/gateway-client";

type RecordValue = Record<string, unknown>;
export type DiscoveryNotice = { id: string; error: boolean; message: string };

export function inputDiscoveryNotices(
  connection: ConnectionState,
  discovery: RecordValue | undefined,
): DiscoveryNotice[] {
  if (connection !== "connected") {
    return [
      {
        id: "connection",
        error: connection === "disconnected",
        message:
          connection === "disconnected"
            ? "输入页与服务的连接已断开，无法确认当前设备状态；下方可能是上次收到的数据。"
            : "正在连接输入服务，尚未取得当前设备状态。",
      },
    ];
  }
  if (!discovery)
    return [
      {
        id: "waiting",
        error: false,
        message:
          "已连接网关，等待输入节点上报设备发现结果；尚不能判断是否有设备。",
      },
    ];
  const drivers = (discovery.drivers ?? []) as RecordValue[];
  const sources = (discovery.sources ?? []) as RecordValue[];
  const notices: DiscoveryNotice[] = [];
  for (const [id, name, empty] of [
    [
      "sdl3-gamepad",
      "SDL 手柄",
      "SDL 未发现可用手柄。请确认手柄已连接或已在主机配对；若主机能看到设备，请检查容器设备访问及 SDL 识别结果。仅看到 USB 或 js 设备文件不能证明 SDL 已成功识别。",
    ],
    [
      "nolo-cv1-hid",
      "NOLO",
      "未发现受支持的 NOLO 设备；未使用 NOLO 时可忽略。",
    ],
  ]) {
    const driver = drivers.find((item) => item.driver_id === id);
    if (!driver) {
      notices.push({
        id,
        error: false,
        message: `${name}驱动尚未上报发现结果，不能判定设备未连接。`,
      });
    } else if (driver.original_error) {
      notices.push({
        id,
        error: true,
        message: `${name}驱动报告错误：${String(driver.original_error)}`,
      });
    } else if (
      !sources.some(
        (source) => source.driver_id === id && source.active === true,
      )
    ) {
      notices.push({ id, error: false, message: empty });
    }
  }
  // Unknown drivers and per-device errors must not disappear from the UI.
  for (const driver of drivers) {
    if (
      driver.original_error &&
      !["sdl3-gamepad", "nolo-cv1-hid"].includes(String(driver.driver_id))
    )
      notices.push({
        id: String(driver.driver_id),
        error: true,
        message: `${String(driver.display_name ?? driver.driver_id)}：${String(driver.original_error)}`,
      });
  }
  for (const source of sources) {
    if (source.original_error)
      notices.push({
        id: `source:${String(source.source_id)}`,
        error: true,
        message: `${String(source.display_name ?? source.source_id)}：${String(source.original_error)}`,
      });
  }
  return notices;
}
