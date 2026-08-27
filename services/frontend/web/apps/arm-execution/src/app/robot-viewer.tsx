"use client";

import type {
  ArmCommand,
  ArmState,
  ExecutionInfo,
  MotionState,
  ParameterValue,
  RobotModelInfo,
} from "@robot/contracts";
import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import URDFLoader from "urdf-loader";
import type { URDFRobot } from "urdf-loader";

type Props = {
  model: RobotModelInfo;
  arm?: ArmState;
  command?: ArmCommand;
  motion?: MotionState;
  execution?: ExecutionInfo;
  parameters: ParameterValue[];
  showLabels: boolean;
};

type Viewer = {
  robot?: URDFRobot;
  commandRobot?: URDFRobot;
  target: THREE.Group;
  labels: THREE.Sprite[];
};

function applyRobotMaterials(
  robot: URDFRobot,
  linkMaterials: RobotModelInfo["visualization"]["link_materials"],
  commandPreview: boolean,
) {
  const links = new Map<THREE.Object3D, string>(
    Object.entries(robot.links).map(([key, link]) => [link, key]),
  );
  let styledMeshes = 0;
  robot.traverse((object) => {
    if (!(object instanceof THREE.Mesh)) return;
    object.castShadow = !commandPreview;
    object.receiveShadow = !commandPreview;
    let owner: THREE.Object3D | null = object;
    while (owner && !links.has(owner)) owner = owner.parent;
    const material = owner ? linkMaterials[links.get(owner) ?? ""] : undefined;
    if (!material && !commandPreview) return;
    object.material = new THREE.MeshStandardMaterial({
      color: commandPreview
        ? 0xe66a3d
        : new THREE.Color(...material!.color_rgb),
      metalness: commandPreview ? 0 : material!.metalness,
      roughness: commandPreview ? 0.75 : material!.roughness,
      opacity: commandPreview ? 0.28 : 1,
      transparent: commandPreview,
      depthWrite: !commandPreview,
    });
    styledMeshes += 1;
  });
  return styledMeshes;
}

function setRobotState(
  robot: URDFRobot | undefined,
  model: RobotModelInfo,
  state: Pick<ArmState, "model_revision" | "joints_rad" | "actuators_rad">,
) {
  if (!robot || state.model_revision !== model.model_revision) return;
  model.joints.forEach((joint, index) =>
    robot.setJointValue(joint.key, state.joints_rad[index] ?? 0),
  );
  model.tool_actuators.forEach((actuator, index) => {
    if (actuator.visualization_joint_key)
      robot.setJointValue(
        actuator.visualization_joint_key,
        state.actuators_rad[index] ?? 0,
      );
  });
}

function labelSprite(text: string) {
  const canvas = document.createElement("canvas");
  canvas.width = 768;
  canvas.height = 160;
  const context = canvas.getContext("2d");
  if (!context) throw new Error("浏览器无法创建关节标签画布");
  context.fillStyle = "rgba(23,32,29,0.9)";
  context.roundRect(0, 0, canvas.width, canvas.height, 24);
  context.fill();
  context.fillStyle = "#f4f1ea";
  context.font = "30px ui-sans-serif, sans-serif";
  text
    .split("\n")
    .forEach((line, index) => context.fillText(line, 24, 50 + index * 42));
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({
      map: texture,
      depthTest: false,
      transparent: true,
    }),
  );
  sprite.scale.set(0.18, 0.0375, 1);
  sprite.position.set(0, 0, 0.035);
  sprite.renderOrder = 10;
  return sprite;
}

function parameterText(key: string, values: ParameterValue[]) {
  return values
    .filter((value) => value.actuator_key === key && value.value != null)
    .map((value) => `${value.field_key}=${value.value}${value.unit}`)
    .join("  ");
}

export function RobotViewer({
  model,
  arm,
  command,
  motion,
  execution,
  parameters,
  showLabels,
}: Props) {
  const mount = useRef<HTMLDivElement>(null);
  const viewer = useRef<Viewer | undefined>(undefined);
  const [modelsReady, setModelsReady] = useState(0);
  const rootPath = model.visualization.root_path;
  const materialsKey = JSON.stringify(model.visualization.link_materials);

  useEffect(() => {
    const element = mount.current;
    if (!element) return;
    const linkMaterials = JSON.parse(
      materialsKey,
    ) as RobotModelInfo["visualization"]["link_materials"];
    const scene = new THREE.Scene();
    scene.background = new THREE.Color("#eef2ed");
    const camera = new THREE.PerspectiveCamera(45, 1, 0.001, 10);
    camera.up.set(0, 0, 1);
    camera.position.set(0.38, -0.48, 0.28);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.shadowMap.enabled = true;
    element.appendChild(renderer.domElement);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0.08);
    controls.update();
    scene.add(new THREE.HemisphereLight(0xffffff, 0x56645e, 2.4));
    const light = new THREE.DirectionalLight(0xffffff, 2.8);
    light.position.set(0.4, -0.3, 0.7);
    scene.add(light);
    const floor = new THREE.GridHelper(0.8, 16, 0x74847c, 0xbcc7c1);
    floor.rotation.x = Math.PI / 2;
    floor.position.z = -0.16;
    scene.add(floor);
    const target = new THREE.Group();
    target.add(new THREE.AxesHelper(0.055));
    target.add(
      new THREE.Mesh(
        new THREE.SphereGeometry(0.008),
        new THREE.MeshBasicMaterial({ color: 0xe66a3d }),
      ),
    );
    target.visible = false;
    scene.add(target);
    viewer.current = { target, labels: [] };

    const resize = () => {
      const width = element.clientWidth;
      const height = element.clientHeight;
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    };
    const observer = new ResizeObserver(resize);
    observer.observe(element);
    resize();
    let animation = 0;
    const render = () => {
      animation = requestAnimationFrame(render);
      renderer.render(scene, camera);
    };
    render();

    let loaded = 0;
    const loadRobot = (commandPreview: boolean) => {
      const manager = new THREE.LoadingManager();
      const loader = new URDFLoader(manager);
      let loadedRobot: URDFRobot | undefined;
      loader.packages = () => "/api/motion/assets";
      manager.onLoad = () => {
        if (!loadedRobot || !viewer.current) return;
        const count = applyRobotMaterials(
          loadedRobot,
          linkMaterials,
          commandPreview,
        );
        if (commandPreview) viewer.current.commandRobot = loadedRobot;
        else viewer.current.robot = loadedRobot;
        loaded += 1;
        element.dataset.modelsLoaded = String(loaded);
        element.dataset.styledMeshes = String(
          Number(element.dataset.styledMeshes ?? 0) + count,
        );
        setModelsReady((value) => value + 1);
      };
      loader.load(
        `/api/motion/assets/${rootPath}`,
        (robot) => {
          loadedRobot = robot;
          robot.visible = !commandPreview;
          scene.add(robot);
        },
        undefined,
        (reason) => {
          element.dataset.error =
            reason instanceof Error ? reason.message : String(reason);
        },
      );
    };
    loadRobot(false);
    loadRobot(true);

    return () => {
      observer.disconnect();
      cancelAnimationFrame(animation);
      controls.dispose();
      scene.traverse((object) => {
        if (!(object instanceof THREE.Mesh)) return;
        object.geometry.dispose();
        const materials = Array.isArray(object.material)
          ? object.material
          : [object.material];
        materials.forEach((material) => material.dispose());
      });
      renderer.dispose();
      renderer.domElement.remove();
      viewer.current = undefined;
    };
  }, [
    materialsKey,
    model.model_revision,
    model.visualization.manifest_hash,
    rootPath,
  ]);

  useEffect(() => {
    if (arm) {
      setRobotState(viewer.current?.robot, model, arm);
      if (mount.current) mount.current.dataset.feedbackReady = "true";
    }
  }, [arm, model, modelsReady]);

  useEffect(() => {
    const robot = viewer.current?.commandRobot;
    if (!robot) return;
    robot.visible = Boolean(command);
    if (command) setRobotState(robot, model, command);
    if (mount.current)
      mount.current.dataset.commandVisible = String(Boolean(command));
  }, [command, model, modelsReady]);

  useEffect(() => {
    const target = viewer.current?.target;
    const pose = motion?.target_tool_pose;
    if (!target) return;
    target.visible = Boolean(pose);
    if (mount.current)
      mount.current.dataset.toolTargetVisible = String(Boolean(pose));
    if (pose) {
      target.position.fromArray(pose.position_m);
      target.quaternion.fromArray(pose.orientation_xyzw);
    }
  }, [motion]);

  useEffect(() => {
    const current = viewer.current;
    const robot = current?.robot;
    if (!current || !robot) return;
    current.labels.forEach((sprite) => {
      sprite.removeFromParent();
      sprite.material.map?.dispose();
      sprite.material.dispose();
    });
    const metadata = [
      ...model.joints.map((joint, index) => ({
        key: joint.key,
        joint: joint.key,
        label: execution?.actuator_labels[index] ?? joint.label,
      })),
      ...model.tool_actuators.map((actuator, index) => ({
        key: actuator.key,
        joint: actuator.visualization_joint_key,
        label:
          execution?.actuator_labels[model.joints.length + index] ??
          actuator.label,
      })),
    ];
    current.labels = metadata.flatMap((item) => {
      const joint = item.joint ? robot.joints[item.joint] : undefined;
      if (!joint) return [];
      const detail = parameterText(item.key, parameters);
      const sprite = labelSprite(
        detail ? `${item.label}\n${detail}` : item.label,
      );
      sprite.visible = showLabels;
      joint.add(sprite);
      return [sprite];
    });
  }, [execution, model, modelsReady, parameters, showLabels]);

  useEffect(() => {
    viewer.current?.labels.forEach((label) => (label.visible = showLabels));
  }, [showLabels]);

  return (
    <div
      className="robot-viewer"
      ref={mount}
      aria-label="机械臂三维反馈与目标预览"
    />
  );
}
