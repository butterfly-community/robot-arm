import { NextResponse } from "next/server";
import { deleteExperience, listExperiences } from "../../../ai/experience";
import { idSchema } from "../../../ai/types";
import { publicError } from "../../../ai/runner";

export const dynamic = "force-dynamic";
export const runtime = "nodejs";

export async function GET(request: Request) {
  try {
    const query = new URL(request.url).searchParams.get("query") ?? "";
    return NextResponse.json(await listExperiences(undefined, query));
  } catch (error) {
    return NextResponse.json(
      { original_error: publicError(error) },
      { status: 400 },
    );
  }
}

export async function DELETE(request: Request) {
  try {
    const { id } = await request.json();
    return NextResponse.json(await deleteExperience(idSchema.parse(id)));
  } catch (error) {
    return NextResponse.json(
      { original_error: publicError(error) },
      { status: 400 },
    );
  }
}
