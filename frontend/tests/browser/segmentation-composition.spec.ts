import { expect, test } from "@playwright/test";
import { inspectLayoutSpacing } from "../../../tools/diagnostics/layout-spacing.mjs";

// Exercise the real page with controlled replies. Every write is intercepted;
// tests never command a robot, change camera configuration or run a real model.
for (const width of [1440, 390])
  test(`manual rectangles combine with both model sources at ${width}px`, async ({
    page,
    request,
  }) => {
    await page.setViewportSize({ width, height: 1000 });
    const snapshot = await (await request.get("/api/perception/state")).json();
    const state = snapshot.values.perception_state;
    Object.assign(state, {
      instances: [],
      manual_regions: [],
      last_segmentation_sequence: null,
      last_scene_sequence: null,
      segmentation_frame: null,
      task_state: "idle",
      calibrated: true,
      task_request_id: null,
      available_models: [
        { id: "text", label: "prompted.pt", prompt_free: false },
        { id: "auto", label: "automatic.pt", prompt_free: true },
      ],
      model: "text",
      classes: ["box"],
      original_error: null,
      color_frame: {
        width: 1920,
        height: 1080,
        encoding: "rgb8",
        frame_id: "optical",
      },
    });
    snapshot.values.camera_state.streaming = true;
    snapshot.values.world_scene = {
      schema_version: 3,
      sequence: 1,
      sample_time_ns: 1,
      frame_id: "base_link",
      objects: [],
      placement_regions: [],
      obstacles: [],
    };
    let publish = () => {};
    await page.routeWebSocket("**/ws/perception", (socket) => {
      publish = () => socket.send(JSON.stringify(snapshot));
      publish();
    });
    await page.routeWebSocket("**/ws/camera-video", () => {});
    const png = await page.evaluate(() => {
      const c = document.createElement("canvas");
      c.width = 1920;
      c.height = 1080;
      c.getContext("2d")!.fillRect(0, 0, c.width, c.height);
      return c.toDataURL().split(",")[1];
    });
    const writes: any[] = [];
    let rejectNext = false;
    await page.route("**/api/**", async (route) => {
      if (route.request().method() !== "POST") {
        if (route.request().url().includes("/assets/"))
          return route.fulfill({
            contentType: "image/png",
            body: Buffer.from(png, "base64"),
          });
        return route.fallback();
      }
      expect(new URL(route.request().url()).pathname).toBe(
        "/api/perception/request",
      );
      const body = route.request().postDataJSON();
      writes.push(body);
      expect(body.action).toBe("refresh");
      if (rejectNext) {
        rejectNext = false;
        return route.fulfill({
          json: { original_error: "标注保存失败（测试）" },
        });
      }
      const edit = body.segmentation_edit;
      if (edit.kind !== "capture")
        expect(body.input_sequence).toBe(state.last_segmentation_sequence);
      if (edit.kind === "capture") {
        state.instances = [];
        state.manual_regions = [];
        state.segmentation_frame = {
          width: 1920,
          height: 1080,
          encoding: "rgb8",
          frame_id: "optical",
        };
      } else if (edit.kind === "manual") {
        // Server acknowledgement is authoritative: JSON key order and harmless
        // numeric round-trip changes must not leave a successfully saved draft dirty.
        state.manual_regions = edit.regions.map((r: any) => ({
          bounding_box_xyxy: r.bounding_box_xyxy.map(
            (v: number) => v + Number.EPSILON * Math.abs(v),
          ),
          label: r.label,
          id: r.id,
        }));
        state.instances = state.instances.filter(
          (i: any) => i.segmentation_source !== "manual",
        );
        state.instances.push(
          ...edit.regions.map((r: any) => ({
            instance_id: `manual:${r.id}`,
            label: r.label,
            bounding_box_xyxy: r.bounding_box_xyxy,
            confidence: 1,
            segmentation_source: "manual",
            mask_asset_key: "mask-manual.png",
            grasp_candidate_count: 0,
          })),
        );
      } else if (edit.kind === "model") {
        state.instances = state.instances.filter(
          (i: any) => i.segmentation_source !== body.model,
        );
        state.instances.push({
          instance_id: `model:${body.model}:1`,
          label: "model object",
          confidence: 0.8,
          bounding_box_xyxy:
            body.model === "auto"
              ? [1100, 100, 1400, 300]
              : [900, 750, 1200, 1000],
          segmentation_source: body.model,
          mask_asset_key: `mask-${body.model}.png`,
          grasp_candidate_count: 0,
        });
      } else if (edit.kind === "remove") {
        state.instances = state.instances.filter(
          (i: any) => !edit.instance_ids.includes(i.instance_id),
        );
        state.manual_regions = state.manual_regions.filter(
          (r: any) => !edit.instance_ids.includes(`manual:${r.id}`),
        );
      } else throw Error(`unexpected edit ${edit.kind}`);
      state.last_segmentation_sequence =
        (state.last_segmentation_sequence ?? 0) + 1;
      state.last_scene_sequence = null;
      state.task_state = "succeeded";
      publish();
      await route.fulfill({ json: { original_error: null, value: state } });
    });
    await page.goto("/perception/");
    for (const title of ["自动分割", "提示词分割", "手动分割"])
      await page
        .getByText(title, { exact: true })
        .locator("xpath=ancestor::button[1]")
        .click();
    await expect(page.getByLabel("自动分割模型", { exact: true })).toHaveValue(
      "auto",
    );
    await expect(
      page.getByLabel("提示词分割模型", { exact: true }),
    ).toHaveValue("text");
    await expect(
      page.getByText("请先载入新分割帧", { exact: true }),
    ).toBeVisible();
    await page
      .getByRole("button", { name: "载入新分割帧", exact: true })
      .click();
    const editor = page.getByRole("img", {
      name: "手动分割框选区域",
      exact: true,
    });
    await expect(editor.locator("image")).toHaveAttribute(
      "href",
      /segmentation-color/,
    );
    await page.getByLabel("新标注名称", { exact: true }).fill("手动区域中文");
    await editor.scrollIntoViewIfNeeded();
    await expect(editor).toBeVisible();
    // Wait for the frozen image, not an arbitrary timer.
    await page.waitForFunction(() => {
      const el = document.querySelector(
        'svg[aria-label="手动分割框选区域"] image',
      );
      if (!el) return false;
      const image = new Image();
      image.src = el.getAttribute("href")!;
      return image.complete;
    });
    const r = (await editor.boundingBox())!;
    await page.mouse.move(r.x + r.width * 0.8, r.y + r.height * 0.7);
    await page.mouse.down();
    await page.mouse.move(r.x + r.width * 0.6, r.y + r.height * 0.5, {
      steps: 4,
    });
    await page.mouse.up();
    await expect(page.getByLabel("标注 1 名称", { exact: true })).toHaveValue(
      "手动区域中文",
    );
    const apply = page.getByRole("button", {
      name: "应用手动标注",
      exact: true,
    });
    rejectNext = true;
    await expect(
      page.getByRole("button", { name: "三维定位", exact: true }),
    ).toBeDisabled();
    await apply.click();
    await expect(apply).toBeEnabled();
    await expect(
      page.getByText("请先应用手动标注，或撤销未应用标注", { exact: true }),
    ).toBeVisible();
    await expect(page.getByLabel("标注 1 名称", { exact: true })).toHaveValue(
      "手动区域中文",
    );
    await page.getByLabel("标注 1 名称", { exact: true }).fill("海绵手工");
    await apply.click();
    await expect(apply).toBeDisabled();
    await expect(
      page.getByRole("button", { name: "三维定位", exact: true }),
    ).toBeEnabled();
    state.calibrated = false;
    publish();
    await expect(
      page.getByRole("button", { name: "三维定位", exact: true }),
    ).toBeDisabled();
    await expect(
      page.getByText("请先确认并应用当前相机的标定", { exact: true }),
    ).toBeVisible();
    state.calibrated = true;
    publish();
    await expect(
      page.getByRole("button", { name: "三维定位", exact: true }),
    ).toBeEnabled();
    await expect(page.getByText("标注已同步", { exact: true })).toBeVisible();
    const box = state.manual_regions[0].bounding_box_xyxy;
    for (const [i, expected] of [1152, 540, 1536, 756].entries())
      expect(Math.abs(box[i] - expected)).toBeLessThanOrEqual(1);
    await page
      .getByLabel("标注 1 名称", { exact: true })
      .fill("未应用的手动草稿");
    await page
      .getByRole("button", { name: "运行自动分割", exact: true })
      .click();
    await expect(page.getByLabel("标注 1 名称", { exact: true })).toHaveValue(
      "未应用的手动草稿",
    );
    expect(state.manual_regions[0].label).toBe("海绵手工");
    await page
      .getByRole("button", { name: "撤销未应用标注", exact: true })
      .click();
    for (const kind of ["提示词", "自动", "提示词"]) {
      const btn = page.getByRole("button", {
        name: `运行${kind}分割`,
        exact: true,
      });
      await btn.click();
      await expect(btn).toBeEnabled();
    }
    expect(state.instances).toHaveLength(3);
    await page.getByText("分割结果", { exact: true }).click();
    const results = page.getByLabel("全部分割结果图", { exact: true });
    await expect(results.locator("image")).toHaveAttribute(
      "href",
      /\/segmentation-color\.png\?v=/,
    );
    await expect(editor.locator("image")).toHaveAttribute(
      "href",
      /\/segmentation-color\.png\?v=/,
    );
    const rows = results.getByRole("button");
    await expect(rows).toHaveCount(3);
    await expect(rows.filter({ hasText: "手动标注" })).toHaveCount(1);
    await expect(rows.filter({ hasText: "prompted.pt" })).toHaveCount(1);
    await expect(rows.filter({ hasText: "automatic.pt" })).toHaveCount(1);
    await expect(results.locator("mask")).toHaveCount(0);
    const resultSection = page
      .getByText("分割结果", { exact: true })
      .locator("xpath=ancestor::div[contains(@class, 'disclosure')][1]");
    await expect(resultSection).toContainText("尚未三维定位");
    const pose = {
      position_m: [0.1, 0.2, 0.03],
      orientation_xyzw: [0, 0, 0, 1],
    };
    state.last_scene_sequence = 42;
    snapshot.values.world_scene = {
      ...snapshot.values.world_scene,
      sequence: 42,
      objects: state.instances.map((i: any) => ({
        object_id: i.instance_id,
        label: i.label,
        pose,
        size_m: [0.03, 0.04, 0.05],
        grasp_candidates: [],
      })),
      placement_regions: [
        {
          region_id: "placement",
          label: "关联放置区",
          source_object_id: state.instances.find(
            (i: any) => i.segmentation_source === "manual",
          ).instance_id,
          pose,
          size_m: [0.06, 0.07, 0.01],
        },
      ],
    };
    publish();
    await expect(resultSection).toContainText("base_link");
    await expect(resultSection).toContainText("3 / 1 / 0");
    await expect(
      editor.getByTitle("海绵手工 · 手动标注", { exact: true }),
    ).toHaveCount(1);
    await expect(
      editor.getByTitle("model object · 提示词 · prompted.pt", { exact: true }),
    ).toHaveCount(0);
    await expect(
      editor.getByTitle("model object · 自动 · automatic.pt", { exact: true }),
    ).toHaveCount(0);
    for (const source of [
      "手动标注",
      "提示词 · prompted.pt",
      "自动 · automatic.pt",
    ]) {
      await rows.filter({ hasText: source }).click();
      const detail = page.getByRole("dialog");
      await expect(detail).toBeVisible();
      await expect(detail).toContainText(source);
      await expect(results.locator("mask")).toHaveCount(0);
      expect((await page.evaluate(inspectLayoutSpacing)).touching).toEqual([]);
      if (source === "手动标注") {
        await expect(detail).toContainText("非模型置信度");
        await expect(detail).toContainText("放置区 · 关联放置区");
      } else await expect(detail).toContainText("80.0%");
      await page.keyboard.press("Escape");
      await expect(detail).toBeHidden();
      await expect(results.locator("mask")).toHaveCount(0);
    }
    await page.reload();
    await expect(page.getByLabel("标注 1 名称", { exact: true })).toHaveValue(
      "海绵手工",
    );
    await expect(rows).toHaveCount(3);
    await rows.filter({ hasText: "海绵手工" }).click();
    rejectNext = true;
    await page
      .getByRole("button", { name: "移除结果 海绵手工", exact: true })
      .click();
    await expect(page.getByRole("dialog")).toBeVisible();
    await expect(rows).toHaveCount(3);
    await page
      .getByRole("button", { name: "移除结果 海绵手工", exact: true })
      .click();
    await expect(rows).toHaveCount(2);
    await expect(page.getByRole("dialog")).toBeHidden();
    await expect(resultSection).toContainText("尚未三维定位");
    await expect(resultSection).not.toContainText("3 / 1 / 0");
    await page
      .getByRole("button", { name: "载入新分割帧", exact: true })
      .click();
    await expect(rows).toHaveCount(0);
    await expect(
      page.getByRole("button", { name: "三维定位", exact: true }),
    ).toBeEnabled();
    // Each model works alone. Neither requires selecting/enabling a global mode.
    for (const [kind, source] of [
      ["自动", "auto"],
      ["提示词", "text"],
    ]) {
      await page
        .getByRole("button", { name: `运行${kind}分割`, exact: true })
        .click();
      await expect(rows).toHaveCount(1);
      expect(state.instances[0].segmentation_source).toBe(source);
      await rows.first().click();
      await expect(page.getByRole("dialog")).toBeVisible();
      state.last_segmentation_sequence += 1;
      publish();
      await expect(page.getByRole("dialog")).toBeHidden();
      await page
        .getByRole("button", { name: `清除${kind}分割结果`, exact: true })
        .click();
      await expect(rows).toHaveCount(0);
    }
    // Independent folds survive reload without disabling the other methods.
    await page
      .getByText("自动分割", { exact: true })
      .locator("xpath=ancestor::button[1]")
      .click();
    await page.reload();
    await expect(
      page
        .getByText("自动分割", { exact: true })
        .locator("xpath=ancestor::button[1]"),
    ).toHaveAttribute("aria-expanded", "false");
    await expect(
      page
        .getByText("提示词分割", { exact: true })
        .locator("xpath=ancestor::button[1]"),
    ).toHaveAttribute("aria-expanded", "true");
    await expect(
      page
        .getByText("手动分割", { exact: true })
        .locator("xpath=ancestor::button[1]"),
    ).toHaveAttribute("aria-expanded", "true");
    expect(
      writes.every((w) => w.action === "refresh" && w.segmentation_edit),
    ).toBe(true);
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth > innerWidth,
      ),
    ).toBe(false);
  });
