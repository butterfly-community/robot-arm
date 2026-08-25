import * as THREE from "https://esm.sh/three@0.180.0?target=es2022";
import URDFLoader from "https://esm.sh/urdf-loader@0.13.1?deps=three@0.180.0&target=es2022";

const URDF_URL = new URL(
  "./models/stararm102_description.urdf",
  import.meta.url,
);
const PACKAGE_ROOT = new URL("./models/", import.meta.url).href;
const ARM_JOINT_NAMES = [
  "joint1",
  "joint2",
  "joint3",
  "joint4",
  "joint5",
  "joint6",
];

function materialForLink(linkName) {
  const gripper = linkName.startsWith("link7");
  const base = linkName === "base_link";
  return new THREE.MeshStandardMaterial({
    color: gripper ? 0x253746 : base ? 0x324c5d : 0xdce7ed,
    metalness: gripper ? 0.42 : 0.16,
    roughness: gripper ? 0.48 : 0.56,
  });
}

function containingLinkName(object) {
  for (let parent = object.parent; parent; parent = parent.parent) {
    if (parent.isURDFLink) return parent.urdfName;
  }
  return "";
}

export async function loadStarArmModel(onProgress = () => {}) {
  const manager = new THREE.LoadingManager();
  manager.onProgress = (_url, loaded, total) => onProgress(loaded, total);
  const geometryLoaded = new Promise((resolve) => {
    manager.onLoad = resolve;
  });
  const loader = new URDFLoader(manager);
  loader.packages = { stararm102_description: PACKAGE_ROOT };
  loader.fetchOptions = { cache: "no-store" };
  const root = await loader.loadAsync(URDF_URL.href);
  await geometryLoaded;

  let meshCount = 0;
  root.traverse((object) => {
    if (!object.isMesh) return;
    object.material = materialForLink(containingLinkName(object));
    object.castShadow = true;
    object.receiveShadow = true;
    meshCount += 1;
  });

  return {
    root,
    meshCount,
    setArmJoints(values) {
      ARM_JOINT_NAMES.forEach((name, index) =>
        root.setJointValue(name, values[index])
      );
    },
    setGripper(value) {
      root.setJointValue("joint7_left", value);
    },
  };
}
