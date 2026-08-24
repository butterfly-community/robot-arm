import * as THREE from "https://cdn.jsdelivr.net/npm/three@0.180.0/+esm";
import { STLLoader } from "https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/loaders/STLLoader.js/+esm";

const URDF_URL = new URL(
  "./models/stararm102_description.urdf",
  import.meta.url,
);
const MESH_BASE_URL = new URL("./models/meshes/", import.meta.url);
const ARM_JOINT_NAMES = [
  "joint1",
  "joint2",
  "joint3",
  "joint4",
  "joint5",
  "joint6",
];

function directChild(element, tagName) {
  return Array.from(element.children).find((child) =>
    child.tagName === tagName
  ) ?? null;
}

function directChildren(element, tagName) {
  return Array.from(element.children).filter((child) =>
    child.tagName === tagName
  );
}

function vectorAttribute(element, name, fallback) {
  const text = element?.getAttribute(name);
  if (!text) return [...fallback];
  const values = text.trim().split(/\s+/).map(Number);
  if (
    values.length !== fallback.length ||
    values.some((value) => !Number.isFinite(value))
  ) {
    throw new Error(`URDF 属性 ${name} 无效：${text}`);
  }
  return values;
}

function quaternionFromRpy([roll, pitch, yaw]) {
  const qx = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(1, 0, 0),
    roll,
  );
  const qy = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 1, 0),
    pitch,
  );
  const qz = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 0, 1),
    yaw,
  );
  return qz.multiply(qy).multiply(qx);
}

function applyOrigin(object, originElement) {
  const [x, y, z] = vectorAttribute(originElement, "xyz", [0, 0, 0]);
  object.position.set(x, y, z);
  object.quaternion.copy(
    quaternionFromRpy(vectorAttribute(originElement, "rpy", [0, 0, 0])),
  );
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

function meshUrl(filename) {
  const name = filename.split("/").pop();
  if (!name || !/^[A-Za-z0-9_.-]+$/.test(name)) {
    throw new Error(`不支持的 URDF 网格路径：${filename}`);
  }
  return new URL(name, MESH_BASE_URL);
}

export async function loadStarArmModel(onProgress = () => {}) {
  const response = await fetch(URDF_URL, { cache: "no-store" });
  if (!response.ok) throw new Error(`URDF 加载失败：HTTP ${response.status}`);
  const documentNode = new DOMParser().parseFromString(
    await response.text(),
    "application/xml",
  );
  const parseError = documentNode.querySelector("parsererror");
  if (parseError) throw new Error(`URDF 解析失败：${parseError.textContent}`);
  const robotElement = documentNode.documentElement;
  if (robotElement.tagName !== "robot") {
    throw new Error("URDF 根节点不是 robot");
  }

  const links = new Map();
  const visualTasks = [];
  const loader = new STLLoader();
  const linkElements = directChildren(robotElement, "link");
  const visualCount = linkElements.reduce(
    (count, link) => count + directChildren(link, "visual").length,
    0,
  );
  let loadedVisuals = 0;

  for (const linkElement of linkElements) {
    const linkName = linkElement.getAttribute("name");
    if (!linkName) throw new Error("URDF link 缺少名称");
    const linkObject = new THREE.Group();
    linkObject.name = linkName;
    links.set(linkName, linkObject);

    for (const visualElement of directChildren(linkElement, "visual")) {
      const geometryElement = directChild(visualElement, "geometry");
      const meshElement = geometryElement &&
        directChild(geometryElement, "mesh");
      const filename = meshElement?.getAttribute("filename");
      if (!filename) continue;
      const visualObject = new THREE.Group();
      visualObject.name = `${linkName}_visual`;
      applyOrigin(visualObject, directChild(visualElement, "origin"));
      visualObject.scale.fromArray(
        vectorAttribute(meshElement, "scale", [1, 1, 1]),
      );
      linkObject.add(visualObject);
      visualTasks.push(
        loader.loadAsync(meshUrl(filename).href).then((geometry) => {
          geometry.computeBoundingSphere();
          const mesh = new THREE.Mesh(geometry, materialForLink(linkName));
          mesh.name = `${linkName}_mesh`;
          mesh.castShadow = true;
          mesh.receiveShadow = true;
          visualObject.add(mesh);
          loadedVisuals += 1;
          onProgress(loadedVisuals, visualCount);
        }),
      );
    }
  }

  const joints = new Map();
  const childLinks = new Set();
  for (const jointElement of directChildren(robotElement, "joint")) {
    const name = jointElement.getAttribute("name");
    const type = jointElement.getAttribute("type") ?? "fixed";
    const parentName = directChild(jointElement, "parent")?.getAttribute(
      "link",
    );
    const childName = directChild(jointElement, "child")?.getAttribute("link");
    const parent = links.get(parentName);
    const child = links.get(childName);
    if (!name || !parent || !child || !childName) {
      throw new Error(`URDF joint ${name ?? "(未命名)"} 的 link 引用无效`);
    }
    childLinks.add(childName);
    const origin = new THREE.Group();
    origin.name = `${name}_origin`;
    applyOrigin(origin, directChild(jointElement, "origin"));
    const motion = new THREE.Group();
    motion.name = `${name}_motion`;
    origin.add(motion);
    motion.add(child);
    parent.add(origin);
    const axis = new THREE.Vector3(
      ...vectorAttribute(directChild(jointElement, "axis"), "xyz", [1, 0, 0]),
    ).normalize();
    const mimicElement = directChild(jointElement, "mimic");
    joints.set(name, {
      name,
      type,
      motion,
      axis,
      value: 0,
      mimic: mimicElement
        ? {
          joint: mimicElement.getAttribute("joint"),
          multiplier: Number(mimicElement.getAttribute("multiplier") ?? 1),
          offset: Number(mimicElement.getAttribute("offset") ?? 0),
        }
        : null,
    });
  }

  const root = new THREE.Group();
  root.name = robotElement.getAttribute("name") ?? "stararm102";
  for (const [name, link] of links) {
    if (!childLinks.has(name)) root.add(link);
  }
  if (root.children.length !== 1) {
    throw new Error(`URDF 应只有一个根 link，实际为 ${root.children.length}`);
  }
  await Promise.all(visualTasks);

  function applyJoint(joint, value) {
    if (!Number.isFinite(value)) return;
    joint.value = value;
    if (joint.type === "revolute" || joint.type === "continuous") {
      joint.motion.quaternion.setFromAxisAngle(joint.axis, value);
    } else if (joint.type === "prismatic") {
      joint.motion.position.copy(joint.axis).multiplyScalar(value);
    }
  }

  function applyMimicJoints() {
    for (const joint of joints.values()) {
      if (!joint.mimic) continue;
      const source = joints.get(joint.mimic.joint);
      if (source) {
        applyJoint(
          joint,
          source.value * joint.mimic.multiplier + joint.mimic.offset,
        );
      }
    }
  }

  return {
    root,
    meshCount: loadedVisuals,
    setArmJoints(values) {
      ARM_JOINT_NAMES.forEach((name, index) =>
        applyJoint(joints.get(name), values[index])
      );
      applyMimicJoints();
    },
    setGripper(value) {
      applyJoint(joints.get("joint7_left"), value);
      applyMimicJoints();
    },
  };
}
