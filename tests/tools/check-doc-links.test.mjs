import assert from "node:assert/strict";
import { execFile } from "node:child_process";
import { copyFile, mkdir, mkdtemp, rm, writeFile } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { promisify } from "node:util";
import test from "node:test";

const exec = promisify(execFile);
const root = fileURLToPath(new URL("../../", import.meta.url));

test("document checks handle unstaged deletion and still reject broken links", async () => {
  await mkdir(path.join(root, "temp"), { recursive: true });
  const fixture = await mkdtemp(path.join(root, "temp/doc-links-test-"));
  try {
    await mkdir(path.join(fixture, "tools"));
    await copyFile(
      path.join(root, "tools/check-doc-links.mjs"),
      path.join(fixture, "tools/check-doc-links.mjs"),
    );
    await exec("git", ["init", "--quiet", fixture]);
    await writeFile(
      path.join(fixture, "README.md"),
      "[present](<with space.md>)\n[external](https://example.invalid/)\n```md\n[example](missing.md)\n```\n",
    );
    await writeFile(path.join(fixture, "with space.md"), "# Present\n");
    await writeFile(path.join(fixture, "removed.md"), "# Removed\n");
    await exec("git", ["add", "README.md", "with space.md", "removed.md"], {
      cwd: fixture,
    });
    await rm(path.join(fixture, "removed.md"));
    const check = () =>
      exec(process.execPath, [path.join(fixture, "tools/check-doc-links.mjs")]);
    assert.match(
      (await check()).stdout,
      /Checked 1 local link paths in 2 Markdown files/,
    );
    await writeFile(path.join(fixture, "new.md"), "[broken](removed.md)\n");
    await assert.rejects(
      check(),
      (error) => error.code === 1 && /new.md:1: removed.md/.test(error.stderr),
    );
    await writeFile(path.join(fixture, "new.md"), "[valid](README.md)\n");
    assert.match(
      (await check()).stdout,
      /Checked 2 local link paths in 3 Markdown files/,
    );
  } finally {
    await rm(fixture, { recursive: true, force: true });
  }
});
