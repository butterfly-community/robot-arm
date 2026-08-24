import { resolvePublicFile } from "./path-utils.ts";

const scriptDir = new URL(".", import.meta.url);
// The shared historical frontend was not duplicated into this frozen archive.
// This source is retained for reference and is not a supported runnable path.
const publicDir = new URL("./public/", scriptDir);
const bridgePath =
  new URL("../build/nolo-controller-stream", scriptDir).pathname;

const hostArg = Deno.args.find((arg) => arg.startsWith("--host="));
const portArg = Deno.args.find((arg) => arg.startsWith("--port="));
const hostname = hostArg?.slice("--host=".length) ?? "127.0.0.1";
const port = Number(portArg?.slice("--port=".length) ?? "8765");
const runtimeJson = Deno.env.get("XR_RUNTIME_JSON") ??
  "/usr/local/share/openxr/1/openxr_monado.json";

if (!Number.isInteger(port) || port < 1 || port > 65535) {
  throw new Error(`invalid port: ${port}`);
}

const encoder = new TextEncoder();
const clients = new Set<WritableStreamDefaultWriter<Uint8Array>>();
let latestFrame: unknown = null;
let bridgeStatus = "正在连接 Monado…";
let stopping = false;
let child: Deno.ChildProcess | null = null;

function event(name: string, value: unknown): Uint8Array {
  return encoder.encode(`event: ${name}\ndata: ${JSON.stringify(value)}\n\n`);
}

function broadcast(name: string, value: unknown) {
  const payload = event(name, value);
  for (const writer of clients) {
    writer.write(payload).catch(() => {
      clients.delete(writer);
    });
  }
}

function setStatus(status: string) {
  bridgeStatus = status;
  broadcast("status", { status });
}

async function pipeStderr(stream: ReadableStream<Uint8Array>) {
  const reader = stream.pipeThrough(new TextDecoderStream()).getReader();
  let buffered = "";
  while (true) {
    const { value, done } = await reader.read();
    if (done) break;
    buffered += value;
    const lines = buffered.split("\n");
    buffered = lines.pop() ?? "";
    for (const line of lines) {
      const message = line.trim();
      if (message.startsWith("streaming ")) setStatus("原始数据已连接");
      if (message.includes("target device not found")) {
        setStatus("未找到右手柄控制器 0");
      }
      if (message.includes("Failed to connect to monado")) {
        setStatus("Monado 服务未运行");
      }
    }
  }
}

async function runBridge() {
  while (!stopping) {
    setStatus("正在连接 Monado…");
    try {
      child = new Deno.Command(bridgePath, {
        args: ["20"],
        env: {
          XR_RUNTIME_JSON: runtimeJson,
        },
        stdout: "piped",
        stderr: "piped",
      }).spawn();

      const stderrTask = pipeStderr(child.stderr);
      const reader = child.stdout.pipeThrough(new TextDecoderStream())
        .getReader();
      let buffered = "";
      while (!stopping) {
        const { value, done } = await reader.read();
        if (done) break;
        buffered += value;
        const lines = buffered.split("\n");
        buffered = lines.pop() ?? "";
        for (const line of lines) {
          if (!line.trim()) continue;
          try {
            latestFrame = JSON.parse(line);
            broadcast("pose", latestFrame);
          } catch {
            setStatus("收到无法解析的姿态数据");
          }
        }
      }
      await child.status;
      await stderrTask;
    } catch (error) {
      setStatus(
        `数据进程启动失败：${
          error instanceof Error ? error.message : String(error)
        }`,
      );
    } finally {
      child = null;
    }
    if (!stopping) {
      setStatus("数据连接已断开，2 秒后重试");
      await new Promise((resolve) => setTimeout(resolve, 2000));
    }
  }
}

function contentType(pathname: string): string {
  if (pathname.endsWith(".html")) return "text/html; charset=utf-8";
  if (pathname.endsWith(".css")) return "text/css; charset=utf-8";
  if (pathname.endsWith(".js")) return "text/javascript; charset=utf-8";
  return "application/octet-stream";
}

async function serveStatic(pathname: string): Promise<Response> {
  const file = resolvePublicFile(pathname, publicDir);
  if (file === null) {
    return new Response("bad path", { status: 400 });
  }
  try {
    const body = await Deno.readFile(file);
    return new Response(body, {
      headers: {
        "content-type": contentType(file.pathname),
        "cache-control": "no-store",
        "x-content-type-options": "nosniff",
        "referrer-policy": "no-referrer",
        "cross-origin-resource-policy": "same-origin",
        "content-security-policy":
          "default-src 'self'; script-src 'self' https://cdn.jsdelivr.net; style-src 'self'; connect-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'",
      },
    });
  } catch {
    return new Response("not found", { status: 404 });
  }
}

function serveEvents(info: Deno.ServeHandlerInfo): Response {
  const stream = new TransformStream<Uint8Array, Uint8Array>();
  const writer = stream.writable.getWriter();
  clients.add(writer);
  writer.write(event("status", { status: bridgeStatus }));
  if (latestFrame !== null) writer.write(event("pose", latestFrame));
  const cleanup = () => {
    clients.delete(writer);
    writer.close().catch(() => {});
  };
  info.completed.then(cleanup, cleanup);
  return new Response(stream.readable, {
    headers: {
      "content-type": "text/event-stream",
      "cache-control": "no-store",
      "connection": "keep-alive",
    },
  });
}

const abortController = new AbortController();
Deno.addSignalListener("SIGINT", () => abortController.abort());
Deno.addSignalListener("SIGTERM", () => abortController.abort());

const bridgeTask = runBridge();
console.log(`NOLO controller viewer: http://${hostname}:${port}/`);

const server = Deno.serve(
  { hostname, port, signal: abortController.signal },
  (request, info) => {
    const url = new URL(request.url);
    if (request.method !== "GET") {
      return new Response("method not allowed", {
        status: 405,
        headers: { allow: "GET" },
      });
    }
    if (url.pathname === "/events") return serveEvents(info);
    if (url.pathname === "/api/status") {
      return Response.json({ status: bridgeStatus, latestFrame }, {
        headers: { "cache-control": "no-store" },
      });
    }
    return serveStatic(url.pathname);
  },
);

await server.finished;
stopping = true;
const processToStop = child as Deno.ChildProcess | null;
if (processToStop !== null) {
  try {
    processToStop.kill("SIGTERM");
  } catch {
    // The bridge may already have exited.
  }
}
await bridgeTask;
