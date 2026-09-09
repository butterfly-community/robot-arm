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
  const missing = [...css.matchAll(/var\((--[\w-]+)/g)]
    .map((match) => match[1])
    .filter((name) => !declared.has(name));
  assert.deepEqual([...new Set(missing)], []);
});
