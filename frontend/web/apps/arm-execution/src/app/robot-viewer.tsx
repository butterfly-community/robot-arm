"use client";

import type {
  ArmCommand,
  ArmState,
  ExecutionInfo,
  ManipulationTaskState,
  MotionState,
  ParameterValue,
  RobotModelInfo,
} from "@robot/contracts";
import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { RoomEnvironment } from "three/examples/jsm/environments/RoomEnvironment.js";
import URDFLoader from "urdf-loader";
import type { URDFRobot } from "urdf-loader";

type Props = {
  model: RobotModelInfo;
  arm?: ArmState;
  command?: ArmCommand;
  motion?: MotionState;
  manipulation?: ManipulationTaskState;
  execution?: ExecutionInfo;
  parameters: ParameterValue[];
  showLabels: boolean;
};

type LinkMaterials = RobotModelInfo["visualization"]["link_materials"];

type LabelMetadata = {
  joint?: string;
  text: string;
};

type Viewer = {
  robot?: URDFRobot;
  commandRobot?: URDFRobot;
  target: THREE.Group;
  pickPoint: THREE.Mesh;
  placePoint: THREE.Mesh;
  labels: THREE.Sprite[];
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  grid: THREE.GridHelper;
};

function applyMaterials(
  robot: URDFRobot,
  materials: LinkMaterials,
  preview: boolean,
) {
  const links = new Map<THREE.Object3D, string>(
    Object.entries(robot.links).map(([key, link]) => [link, key]),
  );
  let count = 0;
  robot.traverse((object) => {
    if (!(object instanceof THREE.Mesh)) return;
    let owner: THREE.Object3D | null = object;
    while (owner && !links.has(owner)) owner = owner.parent;
    const source = owner ? materials[links.get(owner) ?? ""] : undefined;
    if (!source && !preview) return;
    object.material = new THREE.MeshStandardMaterial({
      color: preview ? 0x43d9e6 : new THREE.Color(...source!.color_rgb),
      metalness: preview ? 0.05 : source!.metalness,
      roughness: preview ? 0.65 : source!.roughness,
      opacity: preview ? 0.22 : 1,
      transparent: preview,
      depthWrite: !preview,
      polygonOffset: preview,
      polygonOffsetFactor: preview ? -1 : 0,
      polygonOffsetUnits: preview ? -1 : 0,
    });
    count += 1;
  });
  return count;
}

function setState(
  robot: URDFRobot | undefined,
  model: RobotModelInfo,
  value: Pick<ArmState, "model_revision" | "joints_rad" | "actuators_rad">,
) {
  if (!robot || value.model_revision !== model.model_revision) return;
  model.joints.forEach((joint, index) =>
    robot.setJointValue(joint.key, value.joints_rad[index] ?? 0),
  );
  model.tool_actuators.forEach((actuator, index) => {
    if (actuator.visualization_joint_key)
      robot.setJointValue(
        actuator.visualization_joint_key,
        value.actuators_rad[index] ?? 0,
      );
  });
}

function centerView(viewer: Viewer, robot: URDFRobot) {
  const bounds = new THREE.Box3();
  robot.traverse((object) => {
    if (object instanceof THREE.Mesh) bounds.expandByObject(object);
  });
  if (bounds.isEmpty()) return;
  viewer.grid.position.z = bounds.min.z;
  const center = bounds.getCenter(new THREE.Vector3());
  const delta = center.clone().sub(viewer.controls.target);
  viewer.camera.position.add(delta);
  viewer.controls.target.copy(center);
  viewer.controls.update();
}

function labelSprite(text: string) {
  const canvas = document.createElement("canvas");
  canvas.width = 720;
  canvas.height = 150;
  const context = canvas.getContext("2d");
  if (!context) throw new Error("浏览器无法创建关节标签画布");
  context.fillStyle = "rgba(5,8,12,.92)";
  context.fillRect(0, 0, canvas.width, canvas.height);
  context.strokeStyle = "#2a3845";
  context.strokeRect(1, 1, canvas.width - 2, canvas.height - 2);
  context.fillStyle = "#edf3f7";
  context.font = "28px ui-monospace, monospace";
  text
    .split("\n")
    .forEach((line, index) => context.fillText(line, 22, 48 + index * 42));
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

function scenePoint(color: number) {
  const point = new THREE.Mesh(
    new THREE.SphereGeometry(0.009, 20, 12),
    new THREE.MeshBasicMaterial({ color }),
  );
  point.visible = false;
  return point;
}

export function RobotViewer({
  model,
  arm,
  command,
  motion,
  manipulation,
  execution,
  parameters,
  showLabels,
}: Props) {
  const mount = useRef<HTMLDivElement>(null);
  const viewer = useRef<Viewer | undefined>(undefined);
  const [modelsReady, setModelsReady] = useState(0);
  const rootPath = model.visualization.root_path;
  const materialKey = JSON.stringify(model.visualization.link_materials);
  const labelMetadataKey = JSON.stringify([
    ...model.joints.map((joint, index) => {
      const label = execution?.actuator_labels[index] ?? joint.label;
      const detail = parameterText(joint.key, parameters);
      return {
        joint: joint.key,
        text: detail ? [label, detail].join("\n") : label,
      };
    }),
    ...model.tool_actuators.map((actuator, index) => {
      const label =
        execution?.actuator_labels[model.joints.length + index] ??
        actuator.label;
      const detail = parameterText(actuator.key, parameters);
      return {
        joint: actuator.visualization_joint_key,
        text: detail ? [label, detail].join("\n") : label,
      };
    }),
  ] satisfies LabelMetadata[]);

  useEffect(() => {
    const element = mount.current;
    if (!element) return;
    const materials = JSON.parse(materialKey) as LinkMaterials;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x070a0f);
    scene.fog = new THREE.Fog(0x070a0f, 1.5, 4);
    const camera = new THREE.PerspectiveCamera(42, 1, 0.001, 10);
    camera.up.set(0, 0, 1);
    camera.position.set(0.38, -0.48, 0.28);
    const renderer = new THREE.WebGLRenderer({
      antialias: true,
      powerPreference: "high-performance",
    });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    const environmentGenerator = new THREE.PMREMGenerator(renderer);
    const environment = environmentGenerator.fromScene(new RoomEnvironment());
    environmentGenerator.dispose();
    scene.environment = environment.texture;
    element.appendChild(renderer.domElement);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.set(0, 0, 0.07);
    controls.update();
    scene.add(new THREE.HemisphereLight(0xcceeff, 0x121922, 2.7));
    const light = new THREE.DirectionalLight(0xffffff, 3.1);
    light.position.set(0.4, -0.3, 0.7);
    scene.add(light);
    const grid = new THREE.GridHelper(0.8, 16, 0x315761, 0x16242b);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);
    const target = new THREE.Group();
    target.add(new THREE.AxesHelper(0.055));
    target.add(
      new THREE.Mesh(
        new THREE.SphereGeometry(0.007),
        new THREE.MeshBasicMaterial({ color: 0xf0b45a }),
      ),
    );
    target.visible = false;
    scene.add(target);
    const pickPoint = scenePoint(0xf05a67);
    const placePoint = scenePoint(0x53d18b);
    scene.add(pickPoint, placePoint);
    viewer.current = {
      target,
      pickPoint,
      placePoint,
      labels: [],
      camera,
      controls,
      grid,
    };

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
      controls.update();
      renderer.render(scene, camera);
    };
    render();

    let loaded = 0;
    const loadRobot = (preview: boolean) => {
      const manager = new THREE.LoadingManager();
      const loader = new URDFLoader(manager);
      loader.packages = () => "/api/motion/assets";
      let loadedRobot: URDFRobot | undefined;
      manager.onLoad = () => {
        if (!loadedRobot || !viewer.current) return;
        const styled = applyMaterials(loadedRobot, materials, preview);
        if (preview) viewer.current.commandRobot = loadedRobot;
        else viewer.current.robot = loadedRobot;
        loaded += 1;
        element.dataset.modelsLoaded = String(loaded);
        element.dataset.styledMeshes = String(
          Number(element.dataset.styledMeshes ?? 0) + styled,
        );
        setModelsReady((value) => value + 1);
      };
      loader.load(
        `/api/motion/assets/${rootPath}`,
        (robot) => {
          loadedRobot = robot;
          robot.visible = !preview;
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
      environment.dispose();
      renderer.dispose();
      renderer.domElement.remove();
      viewer.current = undefined;
    };
  }, [materialKey, model.model_revision, rootPath]);

  useEffect(() => {
    if (!arm) return;
    const current = viewer.current;
    setState(current?.robot, model, arm);
    if (mount.current) mount.current.dataset.feedbackReady = "true";
  }, [arm, model, modelsReady]);

  useEffect(() => {
    const current = viewer.current;
    if (current?.robot) centerView(current, current.robot);
  }, [modelsReady]);

  useEffect(() => {
    const robot = viewer.current?.commandRobot;
    if (!robot) return;
    // Three.js Object3D is intentionally mutable.
    // eslint-disable-next-line react-hooks/immutability
    robot.visible = Boolean(command);
    if (command) setState(robot, model, command);
    if (mount.current)
      mount.current.dataset.commandVisible = String(Boolean(command));
  }, [command, model, modelsReady]);

  useEffect(() => {
    const target = viewer.current?.target;
    const pose = motion?.target_tool_pose;
    if (!target) return;
    // Three.js Object3D is intentionally mutable.
    // eslint-disable-next-line react-hooks/immutability
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
    if (!current) return;
    const active =
      manipulation?.state === "planning" || manipulation?.state === "executing";
    const pickTarget = manipulation?.pick_position_m;
    const placeTarget = manipulation?.place_position_m;
    current.pickPoint.visible = Boolean(active && pickTarget);
    current.placePoint.visible = Boolean(active && placeTarget);
    if (pickTarget) {
      current.pickPoint.position.fromArray(pickTarget);
      if (mount.current)
        mount.current.dataset.pickPointZ = pickTarget[2].toFixed(3);
    }
    if (placeTarget) {
      current.placePoint.position.fromArray(placeTarget);
      if (mount.current)
        mount.current.dataset.placePointZ = placeTarget[2].toFixed(3);
    }
    if (mount.current) {
      mount.current.dataset.pickPointVisible = String(
        current.pickPoint.visible,
      );
      mount.current.dataset.placePointVisible = String(
        current.placePoint.visible,
      );
    }
  }, [manipulation, model.model_revision, modelsReady]);

  useEffect(() => {
    const current = viewer.current;
    const robot = current?.robot;
    if (!current || !robot) return;
    current.labels.forEach((sprite) => {
      sprite.removeFromParent();
      sprite.material.map?.dispose();
      sprite.material.dispose();
    });
    const metadata = JSON.parse(labelMetadataKey) as LabelMetadata[];
    current.labels = metadata.flatMap((item) => {
      const joint = item.joint ? robot.joints[item.joint] : undefined;
      if (!joint) return [];
      const sprite = labelSprite(item.text);
      sprite.visible = false;
      joint.add(sprite);
      return [sprite];
    });
  }, [labelMetadataKey, modelsReady]);

  useEffect(() => {
    viewer.current?.labels.forEach((label) => (label.visible = showLabels));
  }, [labelMetadataKey, modelsReady, showLabels]);

  return (
    <div
      className="robot-viewer"
      ref={mount}
      aria-label="机械臂三维反馈与目标预览"
    />
  );
}
