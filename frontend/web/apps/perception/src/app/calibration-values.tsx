import type { ReactNode } from "react";

export function CalibrationValues({
  label,
  children,
}: {
  label: string;
  children: ReactNode;
}) {
  return (
    <div
      className="calibration-values"
      role="region"
      aria-label={label}
      tabIndex={0}
    >
      <div>{children}</div>
    </div>
  );
}
