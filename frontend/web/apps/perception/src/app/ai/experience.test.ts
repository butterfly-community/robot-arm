import { beforeEach, describe, expect, it, vi } from "vitest";
import { type AIRun, experienceInputSchema } from "./types";

const values = vi.hoisted(() => new Map<string, string>());
const beforeWrite = vi.hoisted(() => vi.fn());
vi.mock("./store", () => ({
  db: async () => ({
    hGet: async (_key: string, id: string) => values.get(id),
    eval: async (
      _script: string,
      { arguments: [id, previous, value] }: { arguments: string[] },
    ) => {
      beforeWrite(id);
      if ((values.get(id) ?? "") !== previous) return 0;
      values.set(id, value);
      return 1;
    },
    hVals: async () => [...values.values()],
    hDel: async (_key: string, id: string) => Number(values.delete(id)),
  }),
}));
import {
  deleteExperience,
  experienceIndex,
  listExperiences,
  readExperience,
  saveExperience,
} from "./experience";

function fixture(id = "run-1"): AIRun {
  return {
    id,
    sessionId: `session-${id}`,
    settings: { model: "fixture-model" },
    experienceScope: {
      modelRevision: "arm",
      mode: "vision",
      feedbackSource: "hardware",
    },
    calls: [
      {
        id: "call-1",
        name: "move_tcp_absolute",
        input: {},
        startedAt: 1,
        endedAt: 2,
      },
    ],
    requests: [{ id: `${id}:call-1:0`, state: "failed" }],
  } as AIRun;
}
const note = () =>
  experienceInputSchema.parse({
    id: null,
    title: "IK 与关节构型",
    conditions: "接近目标时 TCP IK 失败",
    lesson: "单点求解失败不能证明整个朝向不可达",
    nextAction: "结合当前反馈预览关节调整，再规划执行",
    assessment: "hypothesis",
    evidenceCallIds: ["call-1"],
  });

describe("persistent experience facts and attributed lessons", () => {
  beforeEach(() => {
    values.clear();
    beforeWrite.mockReset();
  });
  it("excludes old intermediate failure facts from the model catalog without deleting them", async () => {
    const run = fixture();
    const saved = await saveExperience(run, note());
    values.set(
      "legacy-fact",
      JSON.stringify({ ...saved, id: "legacy-fact", reflection: undefined }),
    );
    const items = await listExperiences();
    expect(items).toHaveLength(2);
    expect(experienceIndex(items).map((item) => item.id)).toEqual([saved.id]);
  });
  it("deduplicates lessons and allows later evidence to refute an earlier assessment", async () => {
    const run = fixture();
    const first = await saveExperience(run, note());
    const next = fixture("run-2");
    await saveExperience(next, { ...note(), assessment: "supported" });
    expect(await listExperiences()).toHaveLength(1);
    const changed = await saveExperience(next, {
      ...note(),
      id: first.id,
      assessment: "refuted",
      lesson: "新观测不支持此前判断",
    });
    expect(changed.reflection?.assessment).toBe("refuted");
    expect(changed.sources).toHaveLength(2);
    expect(changed.sources[1].callIds).toEqual(["call-1"]);
    expect(changed.reflection?.model).toBe("fixture-model");
  });
  it("keeps hardware, simulation, mode and robot scopes distinct", async () => {
    const run = fixture();
    const saved = await saveExperience(run, note());
    for (const other of [
      { ...run.experienceScope!, feedbackSource: "simulation" },
      { ...run.experienceScope!, mode: "depth" as const },
      { ...run.experienceScope!, modelRevision: "other-arm" },
    ]) {
      expect(await listExperiences(other)).toEqual([]);
      await expect(readExperience(saved.id, other)).rejects.toThrow("不属于");
      await expect(
        saveExperience(
          { ...run, experienceScope: other },
          { ...note(), id: saved.id },
        ),
      ).rejects.toThrow("不能覆盖");
    }
  });
  it("rejects invented or unfinished evidence, without promoting controller success to grasp proof", async () => {
    const run = fixture();
    await expect(
      saveExperience(run, { ...note(), evidenceCallIds: ["invented"] }),
    ).rejects.toThrow("不存在或未完成");
    run.calls[0].endedAt = undefined;
    await expect(saveExperience(run, note())).rejects.toThrow("不存在或未完成");
    expect(values.size).toBe(0);
    // A user correction can be attributed to this task without tool evidence.
    const saved = await saveExperience(run, { ...note(), evidenceCallIds: [] });
    expect(saved.sources[0]).toEqual({
      runId: run.id,
      sessionId: run.sessionId,
      callIds: [],
    });
    expect(saved.reflection?.assessment).toBe("hypothesis");
  });
  it("searches and deletes without changing stored task history or restoring a deleted update", async () => {
    const run = fixture();
    const saved = await saveExperience(run, note());
    expect(await listExperiences(run.experienceScope, "ik")).toHaveLength(1);
    expect(
      await listExperiences(run.experienceScope, "unrelated"),
    ).toHaveLength(0);
    expect(await deleteExperience(saved.id)).toEqual({ deleted: true });
    expect(await listExperiences()).toEqual([]);
    await expect(
      saveExperience(run, { ...note(), id: saved.id }),
    ).rejects.toThrow("已删除");
    expect(run.calls).toHaveLength(1);
  });
  it("does not resurrect a record deleted between read and write", async () => {
    const run = fixture();
    const saved = await saveExperience(run, note());
    beforeWrite.mockImplementationOnce((id: string) => values.delete(id));
    await expect(
      saveExperience(run, { ...note(), id: saved.id }),
    ).rejects.toThrow("更改或删除");
    expect(await listExperiences()).toEqual([]);
  });
  it("does not overwrite another update made after the original read", async () => {
    const run = fixture();
    const saved = await saveExperience(run, note());
    beforeWrite.mockImplementationOnce((id: string) =>
      values.set(id, JSON.stringify({ ...saved, title: "newer" })),
    );
    await expect(
      saveExperience(run, { ...note(), id: saved.id }),
    ).rejects.toThrow("更改或删除");
    expect((await readExperience(saved.id, run.experienceScope!)).title).toBe(
      "newer",
    );
  });
});
