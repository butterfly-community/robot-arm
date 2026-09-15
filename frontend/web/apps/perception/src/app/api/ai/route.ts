import { after, NextResponse } from "next/server";
import { z } from "zod";
import {
  createRun,
  executeRun,
  listModels,
  checkCapabilities,
  publicError,
  stopRun,
} from "../../ai/runner";
import {
  capabilityChecks,
  runs,
  sessions,
  settings,
  writeJSON,
} from "../../ai/store";
import { idSchema, settingsSchema, startSchema } from "../../ai/types";

export const dynamic = "force-dynamic";
export const runtime = "nodejs";
export async function GET(request: Request) {
  try {
    const id = new URL(request.url).searchParams.get("session");
    return NextResponse.json({
      settings: await settings(),
      keyConfigured: Boolean(process.env.AI_API_KEY),
      checks: await capabilityChecks(),
      sessions: await sessions(),
      runs: id ? await runs(idSchema.parse(id)) : [],
    });
  } catch (error) {
    return NextResponse.json(
      { original_error: publicError(error) },
      { status: 400 },
    );
  }
}
export async function POST(request: Request) {
  try {
    const body = await request.json();
    switch (body.action) {
      case "start": {
        const { run, created } = await createRun(startSchema.parse(body));
        if (created) after(() => executeRun(run));
        return NextResponse.json(run, { status: 202 });
      }
      case "stop":
        return NextResponse.json(await stopRun(idSchema.parse(body.runId)));
      case "settings": {
        const value = settingsSchema.parse(body.settings);
        await writeJSON("settings", value);
        return NextResponse.json(value);
      }
      case "models":
        return NextResponse.json(
          await listModels(settingsSchema.parse(body.settings)),
        );
      case "check":
        return NextResponse.json(
          await checkCapabilities(
            settingsSchema.parse(body.settings),
            body.imageId
              ? z
                  .string()
                  .regex(/^[a-f0-9]{64}$/)
                  .parse(body.imageId)
              : undefined,
          ),
        );
      default:
        throw Error("未知 AI 操作");
    }
  } catch (error) {
    return NextResponse.json(
      { original_error: publicError(error) },
      { status: 400 },
    );
  }
}
