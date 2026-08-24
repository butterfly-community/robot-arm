const publicDirectory = new URL(
  "../src/controller-viewer/public/",
  import.meta.url,
);

function source(name: string): string {
  return Deno.readTextFileSync(new URL(name, publicDirectory));
}

Deno.test("viewer uses the pinned Three.js CDN without a local vendor copy", () => {
  const app = source("app.js");
  if (
    !app.includes(
      'from "https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js"',
    )
  ) {
    throw new Error("pinned Three.js CDN import is missing");
  }
  if (app.includes("vendor/three")) {
    throw new Error("viewer still references a local Three.js vendor copy");
  }
});

Deno.test("viewer exposes both controllers and the head selector", () => {
  const html = source("index.html");
  for (const sourceId of [0, 1, 2]) {
    if (!html.includes(`data-source="${sourceId}"`)) {
      throw new Error(`source selector ${sourceId} is missing`);
    }
  }
});

Deno.test("viewer DOM references resolve and use explicit pose validity", () => {
  const app = source("app.js");
  const html = source("index.html");
  const referencedIds = [...app.matchAll(/\$\("([^"]+)"\)/g)].map((match) =>
    match[1]
  );
  const htmlIds = [...html.matchAll(/\bid="([^"]+)"/g)].map((match) =>
    match[1]
  );
  const missing = [...new Set(referencedIds)].filter((id) =>
    !htmlIds.includes(id)
  );
  const duplicate = htmlIds.filter((id, index) =>
    htmlIds.indexOf(id) !== index
  );
  if (missing.length || duplicate.length) {
    throw new Error(JSON.stringify({ missing, duplicate }));
  }
  if (
    !app.includes("frame.source_online") ||
    app.includes("frame.controller_online")
  ) {
    throw new Error("viewer source-online API contract is stale");
  }
  if (!app.includes("frame.pose_usable === true")) {
    throw new Error(
      "viewer still derives pose validity from compatibility flags",
    );
  }
  if (app.includes("(frame.flags & 0x0f)")) {
    throw new Error(
      "viewer still treats compatibility flags as tracking flags",
    );
  }
  if (!app.includes("/api/pose-calibration/${sourceId}")) {
    throw new Error("pose calibration does not restart Fusion");
  }
  if (!app.includes("FUSION_CALIBRATION_DELAY_MS = 1000")) {
    throw new Error("pose calibration does not ignore the first second");
  }
  if (!app.includes("cancelCalibrationHold();")) {
    throw new Error("source changes do not cancel stale calibration holds");
  }
});
