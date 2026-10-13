import { chooseOption } from "./select-controls.mjs";
// Real page controls only; use discover before choosing device keys and profiles.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir, readFile, writeFile, unlink } from "node:fs/promises";
import { resolve, join } from "node:path";
const [mode, directory, role, source, profile] = process.argv.slice(2);
const titles = {
  external: "外部摄像头 1（左）",
  external_right: "外部摄像头 2（右）",
  wrist: "腕部摄像头 1（左）",
  wrist_right: "腕部摄像头 2（右）",
};
if (
  !["discover", "bind", "unbind", "on", "off", "cycle", "measure"].includes(
    mode,
  ) ||
  !directory
)
  throw Error(
    "Usage: camera-bindings.mjs discover|bind|unbind|on|off|cycle|measure OUTPUT [external|external_right|wrist|wrist_right SOURCE PROFILE]",
  );
const output = resolve(directory);
process.env.TMPDIR = join(output, "browser-temp");
await mkdir(process.env.TMPDIR, { recursive: true });
const netlog = join(output, "browser-netlog.json");
let measurement;
let measurementStart;
let measurementEnd;
const browser = await chromium.launch({
  headless: true,
  args: mode === "measure" ? [`--log-net-log=${netlog}`] : [],
});
try {
  const page = await browser.newPage({
    viewport: { width: 1440, height: 1000 },
  });
  if (process.env.CAMERA_DIAGNOSTIC_FREEZE_STATE === "1") {
    if (process.env.CAMERA_VERIFY_CONTINUITY === "1")
      throw Error("Diagnostic state freeze cannot be used for acceptance");
    await page.addInitScript(() => {
      const NativeSocket = window.WebSocket;
      window.WebSocket = class extends NativeSocket {
        constructor(...args) {
          super(...args);
          this.addEventListener("message", (event) => {
            if (
              window.__cameraFreezeState &&
              !this.url.includes("camera-video")
            ) {
              event.stopImmediatePropagation();
              this.send("next");
            }
          });
        }
      };
    });
  }
  let measuring = false;
  const streams = new Map();
  page.on("websocket", (socket) => {
    const url = socket.url();
    const frames = [];
    streams.set(url, frames);
    let metadata;
    socket.on("framereceived", ({ payload }) => {
      if (typeof payload === "string" && url.includes("camera-video")) {
        metadata = JSON.parse(payload);
      }
      if (measuring)
        frames.push({
          arrival_time_ms: Date.now(),
          bytes: Buffer.byteLength(payload),
          binary: Buffer.isBuffer(payload),
          ...(Buffer.isBuffer(payload) ? metadata : {}),
        });
    });
  });
  await page.goto(
    (process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765") +
      "/perception/",
  );
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常", {
    timeout: 60000,
  });
  await page.getByRole("button", { name: "刷新相机列表", exact: true }).click();
  await expect(
    page.getByRole("button", { name: "刷新相机列表", exact: true }),
  ).toBeEnabled();
  const label = titles[role] ?? titles.external;
  const section = page.getByRole("region", {
    name: `${label}绑定`,
    exact: true,
  });
  if (mode === "measure") {
    const preview = page.getByLabel(`${label}浮动窗口`, { exact: true });
    const expand = preview.getByRole("button", { name: "展开", exact: true });
    if (await expand.isVisible()) await expand.click();
    const video = preview.locator("video");
    await expect
      .poll(() => video.evaluate((v) => v.readyState), { timeout: 30000 })
      .toBeGreaterThanOrEqual(2);
    await page.waitForTimeout(3000);
    if (process.env.CAMERA_DIAGNOSTIC_FREEZE_STATE === "1")
      await page.evaluate(() => {
        window.__cameraFreezeState = true;
      });
    measuring = true;
    measurementStart = Date.now();
    const playback = await video.evaluate(async (v) => {
      const start = performance.now();
      const before = v.getVideoPlaybackQuality();
      const time = v.currentTime;
      let firstPresented;
      let lastPresented;
      let callback;
      let paints = 0;
      let paintCallback;
      const paint = () => {
        paints++;
        paintCallback = requestAnimationFrame(paint);
      };
      paintCallback = requestAnimationFrame(paint);
      const presented = (now, metadata) => {
        firstPresented ??= { time: now, frames: metadata.presentedFrames };
        lastPresented = { time: now, frames: metadata.presentedFrames };
        callback = v.requestVideoFrameCallback(presented);
      };
      callback = v.requestVideoFrameCallback(presented);
      await new Promise((resolve) => setTimeout(resolve, 30000));
      v.cancelVideoFrameCallback(callback);
      cancelAnimationFrame(paintCallback);
      const after = v.getVideoPlaybackQuality();
      const seconds = (performance.now() - start) / 1000;
      const canvas = document.createElement("canvas");
      canvas.width = 64;
      canvas.height = 36;
      const context = canvas.getContext("2d");
      context.drawImage(v, 0, 0, canvas.width, canvas.height);
      const pixels = context.getImageData(
        0,
        0,
        canvas.width,
        canvas.height,
      ).data;
      const meanRgb = [0, 0, 0];
      for (let i = 0; i < pixels.length; i += 4)
        for (let c = 0; c < 3; c++) meanRgb[c] += pixels[i + c];
      return {
        seconds,
        diagnostic_state_frozen: !!window.__cameraFreezeState,
        animation_fps: paints / seconds,
        width: v.videoWidth,
        height: v.videoHeight,
        // totalVideoFrames counts presented OR dropped frames, not successful
        // decoder output. Actual presentation is measured separately below.
        processed_fps:
          (after.totalVideoFrames - before.totalVideoFrames) / seconds,
        dropped_frames: after.droppedVideoFrames - before.droppedVideoFrames,
        presented_fps:
          lastPresented &&
          firstPresented &&
          lastPresented.time > firstPresented.time
            ? ((lastPresented.frames - firstPresented.frames) * 1000) /
              (lastPresented.time - firstPresented.time)
            : 0,
        mean_rgb: meanRgb.map(
          (value) => value / (canvas.width * canvas.height),
        ),
        media_seconds: v.currentTime - time,
        buffered_ahead_seconds: v.buffered.length
          ? v.buffered.end(v.buffered.length - 1) - v.currentTime
          : 0,
      };
    });
    measuring = false;
    measurementEnd = Date.now();
    // Supplementary check of the image endpoint used by AI tools: request a
    // fresh image from the same H.264 stream, outside the bandwidth window.
    const requestedAt = Date.now();
    const snapshot = await page.request.get(
      `${new URL(page.url()).origin}/api/camera/snapshot?role=${role}`,
    );
    expect(snapshot.ok()).toBe(true);
    const snapshotMetadata = JSON.parse(snapshot.headers()["x-camera-frame"]);
    const snapshotBytes = await snapshot.body();
    expect(snapshotMetadata.video_codec).toBe("h264");
    expect(snapshotMetadata.pixel_format).toBe("rgb8");
    expect(snapshotMetadata.received_time_ns / 1e6).toBeGreaterThanOrEqual(
      requestedAt,
    );
    expect(snapshotMetadata.width).toBe(playback.width);
    expect(snapshotMetadata.height).toBe(playback.height);
    const result = {
      playback,
      snapshot: {
        ...snapshotMetadata,
        bytes: snapshotBytes.length,
        latency_ms: Date.now() - requestedAt,
      },
      streams: [...streams].map(([url, frames]) => {
        const packets = frames.filter((f) => f.binary);
        const bytes = frames.reduce((sum, f) => sum + f.bytes, 0);
        const sorted = packets.map((f) => f.bytes).sort((a, b) => a - b);
        const periods = packets.map((f) => f.duration_ms).sort((a, b) => a - b);
        const age = packets
          .map((f) => f.arrival_time_ms - f.received_time_ns / 1e6)
          .sort((a, b) => a - b);
        return {
          url,
          messages: frames.length,
          packets: packets.length,
          codec: packets[0]?.codec,
          frame_period_ms: periods.length
            ? {
                min: periods[0],
                p01: periods[Math.floor(periods.length * 0.01)],
                median: periods[Math.floor(periods.length * 0.5)],
                p99: periods[Math.floor(periods.length * 0.99)],
                max: periods.at(-1),
                shorter_than_60hz: periods.filter((p) => p < 1000 / 60).length,
              }
            : null,
          bytes,
          MB_per_second: bytes / playback.seconds / 1e6,
          Mb_per_second: (bytes * 8) / playback.seconds / 1e6,
          received_fps: packets.length / playback.seconds,
          // Host dequeue -> browser WebSocket event; excludes sensor exposure
          // and any frames already queued inside the device / kernel.
          delivery_age_mean_ms: age.length
            ? age.reduce((sum, value) => sum + value, 0) / age.length
            : null,
          delivery_age_p95_ms: age[Math.floor(age.length * 0.95)] ?? null,
          source_sequence_advance: packets.length
            ? packets.at(-1).sequence - packets[0].sequence
            : null,
          source_frame_gaps: packets
            .slice(1)
            .reduce(
              (sum, packet, index) =>
                sum +
                Math.max(0, packet.sequence - packets[index].sequence - 1),
              0,
            ),
          duplicate_or_reordered_frames: packets
            .slice(1)
            .filter(
              (packet, index) => packet.sequence <= packets[index].sequence,
            ).length,
          source_time_seconds: packets.length
            ? (packets.at(-1).device_time_ns - packets[0].device_time_ns) / 1e9
            : null,
          mean_frame_bytes: packets.length
            ? packets.reduce((sum, f) => sum + f.bytes, 0) / packets.length
            : null,
          p95_frame_bytes: sorted[Math.floor(sorted.length * 0.95)] ?? null,
          maximum_frame_bytes: sorted.at(-1) ?? null,
          largest_message_bytes: Math.max(0, ...frames.map((f) => f.bytes)),
          keyframes: packets.filter((f) => f.keyframe).length,
        };
      }),
    };
    await writeFile(
      join(output, "performance.json"),
      JSON.stringify(result, null, 2),
    );
    measurement = result;
    console.log("MEDIA_PERFORMANCE", JSON.stringify(result));
    expect(playback.processed_fps).toBeGreaterThan(0);
    expect(
      result.streams.some((s) => s.codec === "h264" && s.packets > 0),
    ).toBe(true);
    if (process.env.CAMERA_VERIFY_CONTINUITY === "1") {
      const stream = result.streams.find((s) => s.url.endsWith(`role=${role}`));
      expect(
        stream.source_frame_gaps,
        "device -> codec -> browser missing frames",
      ).toBe(0);
      expect(stream.duplicate_or_reordered_frames).toBe(0);
      expect(playback.dropped_frames, "browser playback drops").toBe(0);
      // Different sample boundaries may include one additional frame. Compare
      // real compositor presentation, not the configured or decoded FPS alone.
      expect(
        Math.abs(
          playback.presented_fps -
            stream.source_sequence_advance / stream.source_time_seconds,
        ),
      ).toBeLessThan(2 / playback.seconds);
    }
  }
  if (mode === "cycle") {
    const state = async () =>
      page.evaluate(
        async () =>
          (await (await fetch("/api/perception/state")).json()).values
            .camera_state.bindings,
      );
    const saved = await state();
    await expect(page.getByText(/共享 USB 带宽不足时/)).toBeVisible();
    for (let round = 0; round < 2; round++) {
      for (const target of saved
        .map((binding) => binding.role)
        .filter((role) => titles[role])) {
        for (const binding of await state()) {
          if (!binding.streaming) continue;
          const name = titles[binding.role];
          const region = page.getByRole("region", {
            name: `${name}绑定`,
            exact: true,
          });
          await region
            .getByRole("button", { name: `关闭${name}采集`, exact: true })
            .click();
          await expect(
            region.getByText("采集已关闭，绑定与分辨率保留", { exact: true }),
          ).toBeVisible();
        }
        const name = titles[target];
        const region = page.getByRole("region", {
          name: `${name}绑定`,
          exact: true,
        });
        const requestedAt = Date.now();
        await region
          .getByRole("button", { name: `开启${name}采集`, exact: true })
          .click();
        await expect(region.getByText("正在采集", { exact: true })).toBeVisible(
          { timeout: 30000 },
        );
        const bindings = await state();
        for (const previous of saved) {
          expect(bindings.find((b) => b.role === previous.role)).toMatchObject({
            source_id: previous.source_id,
            color_profile_key: previous.color_profile_key,
          });
        }
        const current = bindings.find((b) => b.role === target);
        expect(current.last_frame_time_ns / 1e6).toBeGreaterThan(requestedAt);
        expect(bindings.filter((b) => b.streaming).map((b) => b.role)).toEqual([
          target,
        ]);
        const preview = page.getByLabel(`${name}浮动窗口`, { exact: true });
        const expand = preview.getByRole("button", {
          name: "展开",
          exact: true,
        });
        if (await expand.isVisible()) await expand.click();
        const canvas = preview.locator("video");
        await expect
          .poll(() => canvas.evaluate((c) => c.videoWidth))
          .toBe(current.frame.width);
        const pixels = await canvas.evaluate((v) => {
          const c = document.createElement("canvas");
          c.width = v.videoWidth;
          c.height = v.videoHeight;
          c.getContext("2d").drawImage(v, 0, 0);
          return c.toDataURL("image/png").split(",")[1];
        });
        await writeFile(
          join(output, `${round}-${target}.png`),
          Buffer.from(pixels, "base64"),
        );
        console.log("SWITCH_PASS", JSON.stringify(current));
      }
    }
    await page.reload();
    await expect(
      page
        .getByRole("region", { name: "腕部摄像头 1（左）绑定", exact: true })
        .getByText("采集已关闭，绑定与分辨率保留", { exact: true }),
    ).toBeVisible();
    console.log("RELOAD_PRESERVED_CAPTURE_SWITCH");
  }
  if (mode === "on" || mode === "off") {
    await section
      .getByRole("button", {
        name: `${mode === "on" ? "开启" : "关闭"}${label}采集`,
        exact: true,
      })
      .click();
    await expect(
      section.getByText(
        mode === "on" ? "正在采集" : "采集已关闭，绑定与分辨率保留",
        { exact: true },
      ),
    ).toBeVisible({ timeout: 30000 });
  }
  if (mode === "unbind") {
    await section
      .getByRole("button", { name: "解除绑定", exact: true })
      .click();
    await expect(
      section.getByLabel(`${label}来源`, { exact: true }),
    ).toHaveAttribute("data-value", "");
  }
  if (mode === "bind") {
    if (!titles[role] || !source || !profile)
      throw Error("missing binding arguments");
    await chooseOption(
      section.getByLabel(`${label}来源`, { exact: true }),
      source,
    );
    await chooseOption(
      section.getByLabel(`${label}流配置`, { exact: true }),
      profile,
    );
    await section
      .getByRole("button", { name: `保存${label}绑定`, exact: true })
      .click();
    await expect(section.getByText("正在采集", { exact: true })).toBeVisible({
      timeout: 30000,
    });
  }
  if (mode === "bind" || mode === "on") {
    const preview = page.getByLabel(`${label}浮动窗口`, { exact: true });
    await expect(preview).toBeVisible();
    const expand = preview.getByRole("button", { name: "展开", exact: true });
    if (await expand.isVisible()) await expand.click();
    await expect
      .poll(() => preview.locator("video").evaluate((c) => c.videoWidth))
      .toBeGreaterThan(300);
    await preview
      .locator("video")
      .screenshot({ path: join(output, `${role}.png`) });
    const pixels = await preview.locator("video").evaluate((v) => {
      const c = document.createElement("canvas");
      c.width = v.videoWidth;
      c.height = v.videoHeight;
      c.getContext("2d").drawImage(v, 0, 0);
      return c.toDataURL("image/png").split(",")[1];
    });
    await writeFile(
      join(output, `${role}-native.png`),
      Buffer.from(pixels, "base64"),
    );
  }
  // Read the same state the page consumes, only after the UI operation ends.
  const s = await page.evaluate(
    async () =>
      (await (await fetch("/api/perception/state")).json()).values.camera_state,
  );
  console.log(
    JSON.stringify(
      {
        bindings: s.bindings,
        sources: s.available_sources.filter((s) => s.driver_id === "v4l2"),
        error: s.original_error,
      },
      null,
      2,
    ),
  );
  await page.screenshot({ path: join(output, "page.png"), fullPage: true });
} finally {
  await browser.close();
  if (measurement) {
    // Playwright's WebSocket event reports already-INFLATED JSON. Chrome's
    // socket counters measure actual received bytes, including RFC 7692 wire
    // compression, video, HTTP and framing. Never label decoded bytes as wire.
    const log = JSON.parse(await readFile(netlog, "utf8"));
    const offset = Number(log.constants.timeTickOffset);
    const kind = log.constants.logEventTypes.SOCKET_BYTES_RECEIVED;
    const bytes = log.events
      .filter(
        (e) =>
          e.type === kind &&
          Number(e.time) + offset >= measurementStart &&
          Number(e.time) + offset <= measurementEnd,
      )
      .reduce((sum, e) => sum + (e.params?.byte_count ?? 0), 0);
    measurement.browser_wire = {
      bytes,
      MB_per_second: bytes / measurement.playback.seconds / 1e6,
      Mb_per_second: (bytes * 8) / measurement.playback.seconds / 1e6,
    };
    await writeFile(
      join(output, "performance.json"),
      JSON.stringify(measurement, null, 2),
    );
    console.log("BROWSER_WIRE", JSON.stringify(measurement.browser_wire));
    await unlink(netlog);
  }
}
