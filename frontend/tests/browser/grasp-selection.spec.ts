import { expect, test } from "@playwright/test";

// UI contract only: all writes are intercepted, never a physical grasp trial.
test("accepted task selection follows updates without losing a later human edit", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/perception/state")).json();
  const pose = { position_m: [0.2, 0.1, 0.02], orientation_xyzw: [0, 0, 0, 1] };
  const object = (id: string) => ({
    object_id: id,
    label: "box",
    pose,
    size_m: [0.03, 0.03, 0.03],
    confidence: 1,
    grasp_candidates: [{ pose, confidence: 1 }],
  });
  const region = (id: string) => ({
    region_id: id,
    label: "pad",
    pose,
    size_m: [0.1, 0.1, 0.003],
    source_object_id: null,
  });
  snapshot.values.world_scene = {
    schema_version: 3,
    sequence: 1,
    sample_time_ns: 1,
    frame_id: "base_link",
    objects: [object("old-box")],
    placement_regions: [region("old-pad")],
    obstacles: [],
    point_cloud: null,
  };
  snapshot.values.manipulation_state = {
    schema_version: 3,
    request_id: "ui-fixture",
    state: "executing",
    object_id: "old-box",
    placement_region_id: "old-pad",
    pick_position_m: null,
    place_position_m: null,
    solution_count: 1,
    selected_cost: 0,
    stage: "执行抓放",
    original_error: null,
  };
  let publish: (() => void) | undefined;
  await page.routeWebSocket("**/ws/perception", (socket) => {
    publish = () => socket.send(JSON.stringify(snapshot));
    publish();
  });
  await page.route("**/api/**", (route) =>
    route.request().method() === "POST" ? route.abort() : route.fallback(),
  );
  await page.goto("/perception/");
  await page.getByText("抓放场景", { exact: true }).click();
  const target = page.getByLabel("抓取目标", { exact: true });
  const destination = page.getByLabel("放置区域", { exact: true });
  await expect(target).toHaveValue("old-box");
  await expect(destination).toHaveValue("old-pad");
  // Exercise an actual human draft, not only the page's first-option default.
  await target.selectOption("old-box");
  await destination.selectOption("old-pad");
  snapshot.values.world_scene.sequence = 2;
  snapshot.values.world_scene.objects = [object("new-box")];
  snapshot.values.world_scene.placement_regions = [
    region("new-pad"),
    region("other-pad"),
  ];
  snapshot.values.manipulation_state.object_id = "new-box";
  snapshot.values.manipulation_state.placement_region_id = "new-pad";
  publish!();
  await expect(target).toHaveValue("new-box");
  await expect(destination).toHaveValue("new-pad");
  snapshot.values.manipulation_state.state = "succeeded";
  publish!();
  await destination.selectOption("other-pad");
  publish!();
  await expect(destination).toHaveValue("other-pad");
});
