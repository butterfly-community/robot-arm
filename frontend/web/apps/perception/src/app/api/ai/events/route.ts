import { idSchema } from "../../../ai/types";
import { runs } from "../../../ai/store";
import { publicError } from "../../../ai/runner";
export const dynamic = "force-dynamic";
export const runtime = "nodejs";
export async function GET(request: Request) {
  const id = idSchema.parse(new URL(request.url).searchParams.get("session"));
  const encoder = new TextEncoder();
  let closed = false;
  const stream = new ReadableStream({
    async start(controller) {
      let previous = "";
      try {
        while (!closed && !request.signal.aborted) {
          const data = JSON.stringify(await runs(id));
          if (data !== previous) {
            controller.enqueue(encoder.encode(`data: ${data}\n\n`));
            previous = data;
          }
          await new Promise((resolve) => setTimeout(resolve, 500));
        }
      } catch (error) {
        if (!closed && !request.signal.aborted)
          controller.enqueue(
            encoder.encode(
              `event: error\ndata: ${JSON.stringify({ original_error: publicError(error) })}\n\n`,
            ),
          );
      } finally {
        if (!closed) controller.close();
      }
    },
    cancel() {
      closed = true;
    },
  });
  return new Response(stream, {
    headers: {
      "content-type": "text/event-stream",
      "cache-control": "no-cache, no-transform",
      "x-accel-buffering": "no",
    },
  });
}
