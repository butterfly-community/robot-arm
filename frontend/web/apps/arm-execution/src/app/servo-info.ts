import type { ParameterValue } from "@robot/contracts";

export function parameterText(item: ParameterValue | undefined): string {
  if (!item) return "—";
  if (item.original_error) return "读取失败";
  if (item.value == null) return "—";
  if (
    item.field_key === "firmware_version" ||
    item.field_key === "servo_type"
  ) {
    return `0x${item.value.toString(16).toUpperCase().padStart(4, "0")}（原值 ${item.value}）`;
  }
  return String(item.value);
}

const statusLabels = [
  "执行中",
  "指令错误",
  "堵转",
  "电压过高",
  "电压过低",
  "电流错误",
  "功率错误",
  "温度错误",
];
export function servoStatus(status: number): string {
  return (
    statusLabels.filter((_, bit) => (status & (1 << bit)) !== 0).join("、") ||
    "正常"
  );
}

export interface ServoFeedback {
  actuator_key: string;
  position_tenths_degree?: number | null;
  turns?: number | null;
  voltage_mv: number;
  current_ma: number;
  power_mw: number;
  temperature_raw: number;
  status: number;
}
