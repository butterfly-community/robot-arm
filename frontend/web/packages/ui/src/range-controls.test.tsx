import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { RangeControls, type RangeItem } from "./index";

function render(items: RangeItem[]) {
  return renderToStaticMarkup(
    <RangeControls
      items={items}
      values={{}}
      onChange={() => {}}
      onCommit={() => {}}
    />,
  );
}

describe("metadata-driven controls", () => {
  it("renders a non-six-axis model without adding an actuator", () => {
    const html = render(
      Array.from({ length: 4 }, (_, index) => ({
        key: `axis-${index}`,
        label: `Axis ${index}`,
        unit: "rad",
        minimum: -1,
        maximum: 1,
      })),
    );
    expect(html.match(/type="range"/g)).toHaveLength(4);
  });

  it("renders every actuator supplied by metadata", () => {
    const html = render([
      { key: "tool-a", label: "Tool A", unit: "m", minimum: 0, maximum: 1 },
      { key: "tool-b", label: "Tool B", unit: "rad", minimum: -2, maximum: 2 },
    ]);
    expect(html.match(/type="range"/g)).toHaveLength(2);
    expect(html).toContain("Tool A");
    expect(html).toContain("Tool B");
  });
});
