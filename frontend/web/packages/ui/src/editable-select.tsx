"use client";

import { Autocomplete } from "@base-ui/react/autocomplete";

export type SelectSuggestion = { id: string; label: string };

/** Suggestions are optional: selecting and typing both edit the same value. */
export function EditableSelect({
  label,
  value,
  onChange,
  options,
  placeholder,
  onOpen,
  emptyText = "暂无可选项，可直接输入",
}: {
  label: string;
  value: string;
  onChange: (value: string) => void;
  options: SelectSuggestion[];
  placeholder?: string;
  onOpen?: () => void;
  emptyText?: string;
}) {
  return (
    <Autocomplete.Root
      items={options}
      value={value}
      onValueChange={(next, details) => {
        // Escape dismisses suggestions; it must not erase a saved setting.
        if (details.reason !== "escape-key") onChange(next);
      }}
      itemToStringValue={(item) => item.id}
      mode="none"
      modal={false}
      onOpenChange={(open) => {
        if (open) onOpen?.();
      }}
    >
      <Autocomplete.InputGroup className="editable-select">
        <Autocomplete.Input
          className="input"
          aria-label={label}
          placeholder={placeholder}
        />
        <Autocomplete.Trigger
          className="editable-select-trigger"
          aria-label={`选择${label}`}
        >
          <span aria-hidden="true">▾</span>
        </Autocomplete.Trigger>
      </Autocomplete.InputGroup>
      <Autocomplete.Portal>
        <Autocomplete.Positioner
          className="editable-select-positioner"
          sideOffset={6}
        >
          <Autocomplete.Popup className="editable-select-popup">
            <Autocomplete.Empty className="editable-select-empty">
              {emptyText}
            </Autocomplete.Empty>
            <Autocomplete.List>
              {(item: SelectSuggestion) => (
                <Autocomplete.Item
                  key={item.id}
                  value={item}
                  className="editable-select-option"
                >
                  {item.label}
                </Autocomplete.Item>
              )}
            </Autocomplete.List>
          </Autocomplete.Popup>
        </Autocomplete.Positioner>
      </Autocomplete.Portal>
    </Autocomplete.Root>
  );
}
