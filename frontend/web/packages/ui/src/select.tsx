"use client";

import { Select as BaseSelect } from "@base-ui/react/select";
import {
  Children,
  Fragment,
  isValidElement,
  type ReactNode,
  type ButtonHTMLAttributes,
} from "react";

type Option = { value: string; label: ReactNode; disabled: boolean };

function optionsFromChildren(children: ReactNode): Option[] {
  return Children.toArray(children).flatMap((child) => {
    if (
      !isValidElement<{
        value?: string | number;
        children?: ReactNode;
        disabled?: boolean;
      }>(child)
    )
      return [];
    if (child.type === Fragment)
      return optionsFromChildren(child.props.children);
    return [
      {
        value: String(child.props.value ?? ""),
        label: child.props.children,
        disabled: Boolean(child.props.disabled),
      },
    ];
  });
}

/** DOM-based options share the page's rendering and pointer coordinates. */
export function Select({
  value,
  onValueChange,
  children,
  disabled,
  className,
  ...props
}: Omit<
  ButtonHTMLAttributes<HTMLButtonElement>,
  "value" | "onChange" | "children"
> & {
  value: string | number;
  onValueChange: (value: string) => void;
  children: ReactNode;
}) {
  const options = optionsFromChildren(children);
  return (
    <BaseSelect.Root
      value={String(value)}
      items={options}
      disabled={disabled}
      modal={false}
      onValueChange={(next) => {
        if (next !== null) onValueChange(next);
      }}
    >
      <BaseSelect.Trigger
        {...props}
        className={["select-trigger", className].filter(Boolean).join(" ")}
        data-value={String(value)}
      >
        <BaseSelect.Value className="select-value">
          {options.length === 0 ? "暂无可选项" : undefined}
        </BaseSelect.Value>
        <BaseSelect.Icon aria-hidden="true">▾</BaseSelect.Icon>
      </BaseSelect.Trigger>
      <BaseSelect.Portal>
        <BaseSelect.Positioner
          className="select-positioner"
          align="start"
          sideOffset={6}
          alignItemWithTrigger={false}
        >
          <BaseSelect.Popup className="select-popup">
            <BaseSelect.List>
              {options.length === 0 && (
                <BaseSelect.Item disabled className="select-option">
                  <BaseSelect.ItemText>暂无可选项</BaseSelect.ItemText>
                </BaseSelect.Item>
              )}
              {options.map((option) => (
                <BaseSelect.Item
                  key={option.value}
                  value={option.value}
                  disabled={option.disabled}
                  data-value={option.value}
                  className="select-option"
                >
                  <BaseSelect.ItemText>{option.label}</BaseSelect.ItemText>
                  <BaseSelect.ItemIndicator aria-hidden="true">
                    ✓
                  </BaseSelect.ItemIndicator>
                </BaseSelect.Item>
              ))}
            </BaseSelect.List>
          </BaseSelect.Popup>
        </BaseSelect.Positioner>
      </BaseSelect.Portal>
    </BaseSelect.Root>
  );
}
