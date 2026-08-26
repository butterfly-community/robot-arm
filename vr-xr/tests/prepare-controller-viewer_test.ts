import { prepareControllerViewer } from "../tools/prepare-controller-viewer.ts";

Deno.test("viewer model is copied from the supplied patched description", async () => {
  const temporary = await Deno.makeTempDir();
  try {
    const description = `${temporary}/stararm102_description`;
    const publicDirectory = `${temporary}/public`;
    await Deno.mkdir(`${description}/urdf`, { recursive: true });
    await Deno.mkdir(`${description}/meshes`, { recursive: true });
    await Deno.mkdir(`${publicDirectory}/arm-simulator/models`, {
      recursive: true,
    });
    await Deno.writeTextFile(
      `${description}/urdf/stararm102_description.urdf`,
      '<robot name="patched-vendor-model" />',
    );
    await Deno.writeTextFile(`${description}/meshes/link1.STL`, "patched mesh");
    await Deno.writeTextFile(
      `${publicDirectory}/arm-simulator/models/stale.STL`,
      "stale",
    );

    await prepareControllerViewer(description, publicDirectory);

    const output = `${publicDirectory}/arm-simulator/models`;
    if (
      await Deno.readTextFile(`${output}/stararm102_description.urdf`) !==
        '<robot name="patched-vendor-model" />' ||
      await Deno.readTextFile(`${output}/meshes/link1.STL`) !== "patched mesh"
    ) {
      throw new Error("generated viewer does not contain the supplied model");
    }
    try {
      await Deno.stat(`${output}/stale.STL`);
      throw new Error("stale viewer model was retained");
    } catch (error) {
      if (!(error instanceof Deno.errors.NotFound)) throw error;
    }
  } finally {
    await Deno.remove(temporary, { recursive: true });
  }
});
