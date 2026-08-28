import { defineConfig } from "@playwright/test";

export default defineConfig({
  testDir: "./tests/browser",
  fullyParallel: false,
  workers: 1,
  reporter: "line",
  use: {
    baseURL: process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765",
    trace: "retain-on-failure",
  },
});
