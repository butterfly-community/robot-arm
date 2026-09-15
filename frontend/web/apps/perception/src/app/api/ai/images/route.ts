import { NextResponse } from "next/server";
import { putImage } from "../../../ai/store";
import { publicError } from "../../../ai/runner";
export const runtime = "nodejs";
export async function POST(request: Request) {
  try {
    const file = (await request.formData()).get("image");
    if (!(file instanceof File)) throw Error("请选择图片文件");
    return NextResponse.json(
      await putImage(Buffer.from(await file.arrayBuffer())),
    );
  } catch (error) {
    return NextResponse.json(
      { original_error: publicError(error) },
      { status: 400 },
    );
  }
}
