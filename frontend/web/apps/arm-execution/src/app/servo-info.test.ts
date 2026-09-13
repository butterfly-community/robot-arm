import { describe, it, expect } from "vitest";
import { parameterText, servoStatus } from "./servo-info";

describe("servo information", () => {
  it("retains firmware wire encoding instead of confusing integer and Vxxx", () => {
    expect(
      parameterText({
        actuator_key: "joint5",
        field_key: "firmware_version",
        value: 0x225,
        unit: "",
        read_time_ns: 1,
      }),
    ).toBe("0x0225（原值 549）");
  });
  it("never renders a failed field as a successful old value", () => {
    expect(
      parameterText({
        actuator_key: "joint5",
        field_key: "firmware_version",
        value: 1,
        unit: "",
        read_time_ns: 1,
        original_error: "timeout",
      }),
    ).toBe("读取失败");
    expect(parameterText(undefined)).toBe("—");
  });
  it("decodes all vendor status bits without treating normal motion as a fault", () => {
    expect(servoStatus(0)).toBe("正常");
    expect(servoStatus(1)).toBe("执行中");
    expect(servoStatus(6)).toBe("指令错误、堵转");
  });
});
