import { defineConfig } from "@playwright/test";
import { mkdirSync } from "node:fs";
import { resolve } from "node:path";

// Browser profiles must also stay in project temp, not the system /tmp.
// Keep them outside outputDir, which Playwright clears when starting a run.
process.env.TMPDIR = resolve(__dirname, "../temp/playwright-browser");
mkdirSync(process.env.TMPDIR, { recursive: true });

export default defineConfig({
  testDir: "./tests/browser",
  outputDir: "../temp/playwright",
  fullyParallel: false,
  workers: 1,
  reporter: "line",
  use: {
    baseURL: process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765",
    // Raw camera WebSocket frames make traces grow by gigabytes. Keep a
    // failure screenshot instead; the tests still exercise the real stream.
    screenshot: "only-on-failure",
  },
});
