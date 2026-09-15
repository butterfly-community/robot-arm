import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";

test("shared CSS references declared tokens, not removed migration names", () => {
  const css = readFileSync(
    new URL("../../frontend/web/packages/ui/src/globals.css", import.meta.url),
    "utf8",
  );
  const declared = new Set(
    [...css.matchAll(/(--[\w-]+)\s*:/g)].map((match) => match[1]),
  );
  // Runtime values supplied by VirtualFeedback (inline style) and Radix.
  declared.add("--feedback-value");
  declared.add("--radix-collapsible-content-height");
  // Base UI Autocomplete.Positioner measures these at runtime.
  // https://base-ui.com/react/components/autocomplete#positioner
  declared.add("--anchor-width");
  declared.add("--available-width");
  declared.add("--available-height");
  const missing = [...css.matchAll(/var\((--[\w-]+)/g)]
    .map((match) => match[1])
    .filter((name) => !declared.has(name));
  assert.deepEqual([...new Set(missing)], []);
});
