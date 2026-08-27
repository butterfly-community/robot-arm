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
const CONTROL_JOINT_NAMES = [...ARM_JOINT_NAMES, "joint7_left"];
const JOINT_LABELS = [
  ["joint1", "J1 / ID 0"],
  ["joint2", "J2 / ID 1"],
  ["joint3", "J3 / ID 2"],
  ["joint4", "J4 / ID 3"],
  ["joint5", "J5 / ID 4"],
  ["joint6", "J6 / ID 5"],
  ["joint7_left", "夹爪 / ID 6"],
];

function jointLabel(title) {
  const canvas = document.createElement("canvas");
  canvas.width = 640;
  canvas.height = 144;
  const context = canvas.getContext("2d");
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({
      map: texture,
      transparent: true,
      depthTest: false,
      depthWrite: false,
    }),
  );
  sprite.name = `servo-label-${title}`;
  sprite.center.set(0.5, 0);
  sprite.scale.set(0.112, 0.0252, 1);
  sprite.renderOrder = 1000;
  function setParameters(parameters) {
    context.clearRect(0, 0, canvas.width, canvas.height);
    context.fillStyle = "rgba(5, 18, 28, 0.88)";
    context.fillRect(2, 2, canvas.width - 4, canvas.height - 4);
    context.strokeStyle = "rgba(71, 216, 235, 0.9)";
    context.lineWidth = 4;
    context.strokeRect(2, 2, canvas.width - 4, canvas.height - 4);
    context.fillStyle = "#e8fbff";
    context.font = '700 40px Inter, "Noto Sans SC", sans-serif';
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText(title, canvas.width / 2, 48);
    context.fillStyle = "#8fe7f2";
    context.font = '600 27px Inter, "Noto Sans SC", sans-serif';
    context.fillText(parameters, canvas.width / 2, 103);
    texture.needsUpdate = true;
  }
  setParameters("参数未读取");
  return { sprite, setParameters };
}

function addJointLabels(root) {
  const labels = [];
  for (const [jointName, title] of JOINT_LABELS) {
    const joint = root.joints[jointName];
    if (!joint) throw new Error(`${jointName} 缺少 ID 标注锚点`);
    const label = jointLabel(title);
    joint.add(label.sprite);
    labels.push(label);
  }
  return labels;
}

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
  const jointLimitsRad = CONTROL_JOINT_NAMES.map((name) => {
    const limit = root.joints[name]?.limit;
    if (!Number.isFinite(limit?.lower) || !Number.isFinite(limit?.upper)) {
      throw new Error(`${name} 缺少 URDF 关节范围`);
    }
    return [limit.lower, limit.upper];
  });
  const jointLabels = addJointLabels(root);

  return {
    root,
    meshCount,
    jointLimitsRad,
    setArmJoints(values) {
      ARM_JOINT_NAMES.forEach((name, index) =>
        root.setJointValue(name, values[index])
      );
    },
    setGripper(value) {
      root.setJointValue("joint7_left", value);
    },
    setJointLabelsVisible(visible) {
      jointLabels.forEach((label) => label.sprite.visible = visible);
    },
    setJointParameters(parameters) {
      jointLabels.forEach((label, id) => {
        const value = Array.isArray(parameters)
          ? parameters.find((entry) => entry?.id === id)
          : null;
        label.setParameters(
          value
            ? `Kp ${value.kp} · Ki ${value.ki} · Kd ${value.kd} · DZ ${value.dead_zone}`
            : "参数未读取",
        );
      });
    },
  };
}
