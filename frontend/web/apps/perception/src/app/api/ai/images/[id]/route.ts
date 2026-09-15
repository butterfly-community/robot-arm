import { getImage } from "../../../../ai/store";
export const runtime = "nodejs";
export async function GET(
  _: Request,
  context: { params: Promise<{ id: string }> },
) {
  try {
    const image = await getImage((await context.params).id);
    return new Response(new Uint8Array(image.bytes), {
      headers: {
        "content-type": image.mediaType,
        "cache-control": "private, max-age=31536000, immutable",
      },
    });
  } catch {
    return new Response("历史图片不可用", { status: 404 });
  }
}
