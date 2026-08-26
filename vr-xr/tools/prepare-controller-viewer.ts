const URDF_NAME = "stararm102_description.urdf";

function joinPath(parent: string, child: string): string {
  return `${parent.replace(/\/+$/, "")}/${child.replace(/^\/+/, "")}`;
}

async function copyDirectory(source: string, destination: string) {
  await Deno.mkdir(destination, { recursive: true });
  for await (const entry of Deno.readDir(source)) {
    const sourcePath = joinPath(source, entry.name);
    const destinationPath = joinPath(destination, entry.name);
    if (entry.isDirectory) {
      await copyDirectory(sourcePath, destinationPath);
    } else if (entry.isFile) {
      await Deno.copyFile(sourcePath, destinationPath);
    }
  }
}

export async function prepareControllerViewer(
  patchedDescription: string,
  publicDirectory: string,
) {
  const sourceUrdf = joinPath(patchedDescription, `urdf/${URDF_NAME}`);
  const sourceMeshes = joinPath(patchedDescription, "meshes");
  const destination = joinPath(publicDirectory, "arm-simulator/models");

  await Deno.remove(destination, { recursive: true }).catch((error) => {
    if (!(error instanceof Deno.errors.NotFound)) throw error;
  });
  await Deno.mkdir(destination, { recursive: true });
  await Deno.copyFile(sourceUrdf, joinPath(destination, URDF_NAME));
  await copyDirectory(sourceMeshes, joinPath(destination, "meshes"));
}

if (import.meta.main) {
  if (Deno.args.length !== 2) {
    throw new Error(
      "用法：prepare-controller-viewer.ts <补丁后的 stararm102_description> <网页 public 目录>",
    );
  }
  await prepareControllerViewer(Deno.args[0], Deno.args[1]);
}
