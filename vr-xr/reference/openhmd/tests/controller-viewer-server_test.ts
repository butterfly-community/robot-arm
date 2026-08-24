import { resolvePublicFile } from "../openxr-bridge/path-utils.ts";

function assert(condition: boolean, message: string) {
  if (!condition) throw new Error(message);
}

const publicDirectory = new URL("file:///srv/controller-viewer/public/");

Deno.test("static root resolves to index inside public directory", () => {
  const result = resolvePublicFile("/", publicDirectory);
  assert(
    result?.href === "file:///srv/controller-viewer/public/index.html",
    `unexpected result: ${result?.href}`,
  );
});

Deno.test("static nested asset remains inside public directory", () => {
  const result = resolvePublicFile(
    "/pose-math.js",
    publicDirectory,
  );
  assert(
    result?.href ===
      "file:///srv/controller-viewer/public/pose-math.js",
    `unexpected result: ${result?.href}`,
  );
});

Deno.test("static path rejects absolute and traversal forms", () => {
  for (
    const pathname of [
      "//etc/passwd",
      "/../etc/passwd",
      "/vendor/../../etc/passwd",
      "/%2e%2e/etc/passwd",
      "/vendor\\three.module.js",
    ]
  ) {
    assert(
      resolvePublicFile(pathname, publicDirectory) === null,
      `unsafe path accepted: ${pathname}`,
    );
  }
});
