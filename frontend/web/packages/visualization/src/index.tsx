"use client";

import type {
  AbsolutePoseFrame,
  TransformedControlFrame,
} from "@robot/contracts";
import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

type Vector3Tuple = [number, number, number];
type QuaternionTuple = [number, number, number, number];
type CameraView = "perspective" | "top" | "front" | "right";

export type PoseVisualization = {
  position_m: Vector3Tuple;
  orientation_xyzw: QuaternionTuple;
};

type Props = {
  pose?: AbsolutePoseFrame | PoseVisualization;
  poseCoordinates?: "scene" | "robot";
  active?: boolean;
  className?: string;
  ariaLabel?: string;
};

type SceneState = {
  device: THREE.Group;
  orientationDevice: THREE.Group;
  vector: THREE.Line;
  projection: THREE.Line;
  floorMarker: THREE.Mesh;
  displacementLabel: HTMLDivElement;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
};

function createFaceMaterial(label: string) {
  const canvas = document.createElement("canvas");
  canvas.width = 512;
  canvas.height = 256;
  const context = canvas.getContext("2d");
  if (!context) throw new Error("无法创建姿态标记纹理");
  context.fillStyle = "#17212b";
  context.fillRect(0, 0, canvas.width, canvas.height);
  context.strokeStyle = "#52616c";
  context.lineWidth = 10;
  context.strokeRect(5, 5, canvas.width - 10, canvas.height - 10);
  context.fillStyle = "#f4f7fa";
  context.font = "700 112px sans-serif";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText(label, canvas.width / 2, canvas.height / 2 + 4);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return new THREE.MeshStandardMaterial({
    map: texture,
    metalness: 0,
    roughness: 0.62,
  });
}

function createOrientationBlock() {
  const group = new THREE.Group();
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.08, 0.13), [
    createFaceMaterial("右"),
    createFaceMaterial("左"),
    createFaceMaterial("上"),
    createFaceMaterial("下"),
    createFaceMaterial("后"),
    createFaceMaterial("前"),
  ]);
  group.add(body);
  for (const [direction, color] of [
    [new THREE.Vector3(1, 0, 0), 0xff5a5f],
    [new THREE.Vector3(0, 1, 0), 0x4bd37b],
    [new THREE.Vector3(0, 0, 1), 0x5794ff],
  ] as const) {
    const length = 0.27;
    const headLength = 0.035;
    const material = new THREE.MeshBasicMaterial({ color, depthTest: false });
    const rotation = new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      direction,
    );
    const shaft = new THREE.Mesh(
      new THREE.CylinderGeometry(0.004, 0.004, length - headLength, 12),
      material,
    );
    shaft.position.copy(direction).multiplyScalar((length - headLength) / 2);
    shaft.quaternion.copy(rotation);
    shaft.renderOrder = 3;
    group.add(shaft);
    const head = new THREE.Mesh(
      new THREE.ConeGeometry(0.014, headLength, 16),
      material,
    );
    head.position.copy(direction).multiplyScalar(length - headLength / 2);
    head.quaternion.copy(rotation);
    head.renderOrder = 3;
    group.add(head);
  }
  const center = new THREE.Mesh(
    new THREE.SphereGeometry(0.009, 16, 12),
    new THREE.MeshBasicMaterial({ color: 0xf4f7fa, depthTest: false }),
  );
  center.renderOrder = 4;
  group.add(center);
  return group;
}

const cameraViews: Array<{ id: CameraView; label: string }> = [
  { id: "perspective", label: "透视" },
  { id: "top", label: "俯视" },
  { id: "front", label: "正视" },
  { id: "right", label: "右视" },
];

export function robotToScenePosition([
  forward,
  left,
  up,
]: Vector3Tuple): Vector3Tuple {
  return [-left, up, -forward];
}

export function robotToSceneOrientation(
  orientation: QuaternionTuple,
): THREE.Quaternion {
  const source = new THREE.Quaternion().fromArray(orientation).normalize();
  const basis = new THREE.Matrix4().set(
    0,
    -1,
    0,
    0,
    0,
    0,
    1,
    0,
    -1,
    0,
    0,
    0,
    0,
    0,
    0,
    1,
  );
  const sourceRotation = new THREE.Matrix4().makeRotationFromQuaternion(source);
  const mapped = basis
    .clone()
    .multiply(sourceRotation)
    .multiply(basis.clone().invert());
  return new THREE.Quaternion().setFromRotationMatrix(mapped).normalize();
}

export function relativeMotionPose(
  motion: TransformedControlFrame,
): PoseVisualization {
  return {
    position_m: motion.translation_m,
    orientation_xyzw: new THREE.Quaternion()
      .setFromEuler(
        new THREE.Euler(
          motion.front_pitch_rad,
          motion.horizontal_arc_rad,
          0,
          "XYZ",
        ),
      )
      .toArray(),
  };
}

function lineGeometry(points: THREE.Vector3[]) {
  return new THREE.BufferGeometry().setFromPoints(points);
}

export function PoseViewer({
  pose,
  poseCoordinates = "scene",
  active = false,
  className,
  ariaLabel = "三维空间位置和设备自身姿态",
}: Props) {
  const mount = useRef<HTMLDivElement>(null);
  const state = useRef<SceneState | undefined>(undefined);
  const [cameraView, setCameraView] = useState<CameraView>("perspective");

  const selectCameraView = (view: CameraView) => {
    const current = state.current;
    if (!current) return;
    const presets: Record<
      CameraView,
      { offset: Vector3Tuple; up: Vector3Tuple }
    > = {
      perspective: { offset: [1.05, 0.72, 1.15], up: [0, 1, 0] },
      top: { offset: [0, 1.65, 0], up: [0, 0, 1] },
      front: { offset: [0, 0, 1.65], up: [0, 1, 0] },
      right: { offset: [1.65, 0, 0], up: [0, 1, 0] },
    };
    const preset = presets[view];
    current.camera.up.set(...preset.up);
    current.camera.position
      .copy(current.controls.target)
      .add(new THREE.Vector3(...preset.offset));
    current.camera.lookAt(current.controls.target);
    current.controls.update();
    setCameraView(view);
  };

  useEffect(() => {
    const element = mount.current;
    if (!element) return;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x070a0f);
    scene.fog = new THREE.Fog(0x070a0f, 2.5, 7);
    const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 20);
    camera.up.set(0, 1, 0);
    camera.position.set(1.05, 0.72, 1.15);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    element.appendChild(renderer.domElement);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.set(0, 0.08, 0);
    controls.update();

    scene.add(new THREE.HemisphereLight(0xdce8ee, 0x111820, 1.3));
    const key = new THREE.DirectionalLight(0xffffff, 1.7);
    key.position.set(0.7, 1.1, 0.8);
    scene.add(key);
    const rim = new THREE.PointLight(0x43d9e6, 1.2, 3);
    rim.position.set(-0.8, 0.45, -0.7);
    scene.add(rim);

    const grid = new THREE.GridHelper(2, 20, 0x315761, 0x16242b);
    scene.add(grid);
    const origin = new THREE.Mesh(
      new THREE.RingGeometry(0.035, 0.052, 40),
      new THREE.MeshBasicMaterial({ color: 0x43d9e6, side: THREE.DoubleSide }),
    );
    origin.rotation.x = -Math.PI / 2;
    origin.position.y = 0.003;
    scene.add(origin);

    const device = createOrientationBlock();
    scene.add(device);
    const vector = new THREE.Line(
      lineGeometry([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineBasicMaterial({ color: 0x43d9e6 }),
    );
    scene.add(vector);
    const projection = new THREE.Line(
      lineGeometry([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineDashedMaterial({
        color: 0x52616c,
        dashSize: 0.025,
        gapSize: 0.018,
      }),
    );
    projection.computeLineDistances();
    scene.add(projection);
    const floorMarker = new THREE.Mesh(
      new THREE.RingGeometry(0.018, 0.027, 28),
      new THREE.MeshBasicMaterial({ color: 0x7b8a94, side: THREE.DoubleSide }),
    );
    floorMarker.rotation.x = -Math.PI / 2;
    scene.add(floorMarker);

    const orientationScene = new THREE.Scene();
    orientationScene.background = new THREE.Color(0x0b1017);
    orientationScene.add(new THREE.HemisphereLight(0xffffff, 0x182029, 2.6));
    const orientationKey = new THREE.DirectionalLight(0xffffff, 2.8);
    orientationKey.position.set(1, 1, 1);
    orientationScene.add(orientationKey);
    const orientationDevice = createOrientationBlock();
    orientationScene.add(orientationDevice);
    const orientationCamera = new THREE.PerspectiveCamera(34, 1, 0.01, 5);
    orientationCamera.position.set(0.55, 0.4, 0.7);
    orientationCamera.lookAt(0, 0, 0);

    const displacementLabel = document.createElement("div");
    displacementLabel.className = "pose-displacement";
    element.appendChild(displacementLabel);
    state.current = {
      device,
      orientationDevice,
      vector,
      projection,
      floorMarker,
      displacementLabel,
      camera,
      controls,
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
      const width = element.clientWidth;
      const height = element.clientHeight;
      renderer.setScissorTest(false);
      renderer.setViewport(0, 0, width, height);
      renderer.render(scene, camera);
      const insetWidth = Math.min(230, Math.max(150, width * 0.28));
      const insetHeight = Math.min(180, Math.max(120, height * 0.32));
      const insetX = width - insetWidth - 14;
      const insetY = 14;
      renderer.clearDepth();
      renderer.setScissorTest(true);
      renderer.setScissor(insetX, insetY, insetWidth, insetHeight);
      renderer.setViewport(insetX, insetY, insetWidth, insetHeight);
      orientationCamera.aspect = insetWidth / insetHeight;
      orientationCamera.updateProjectionMatrix();
      renderer.render(orientationScene, orientationCamera);
      renderer.setScissorTest(false);
    };
    render();

    return () => {
      observer.disconnect();
      cancelAnimationFrame(animation);
      controls.dispose();
      scene.traverse((object) => {
        if (!(object instanceof THREE.Mesh || object instanceof THREE.Line))
          return;
        object.geometry.dispose();
        const materials = Array.isArray(object.material)
          ? object.material
          : [object.material];
        materials.forEach((material) => {
          if (material instanceof THREE.MeshStandardMaterial) {
            material.map?.dispose();
          }
          material.dispose();
        });
      });
      orientationScene.traverse((object) => {
        if (!(object instanceof THREE.Mesh || object instanceof THREE.Line))
          return;
        object.geometry.dispose();
        const materials = Array.isArray(object.material)
          ? object.material
          : [object.material];
        materials.forEach((material) => {
          if (material instanceof THREE.MeshStandardMaterial) {
            material.map?.dispose();
          }
          material.dispose();
        });
      });
      renderer.dispose();
      renderer.domElement.remove();
      displacementLabel.remove();
      state.current = undefined;
    };
  }, []);

  useEffect(() => {
    const current = state.current;
    const element = mount.current;
    if (!current || !element) return;
    const positionValid =
      pose != null &&
      pose.position_m.every(Number.isFinite) &&
      (!("flags" in pose) || pose.flags.position_valid);
    const orientationValid =
      pose != null &&
      pose.orientation_xyzw.every(Number.isFinite) &&
      (!("flags" in pose) || pose.flags.orientation_valid);
    const valid = positionValid || orientationValid;
    current.device.visible = valid;
    current.orientationDevice.visible = valid;
    current.vector.visible = valid;
    current.projection.visible = valid;
    current.floorMarker.visible = valid;
    element.dataset.poseReady = String(valid);
    element.dataset.controlActive = String(active);
    if (!valid) {
      current.displacementLabel.textContent = "等待有效空间数据";
      return;
    }
    const position =
      positionValid && pose
        ? poseCoordinates === "robot"
          ? robotToScenePosition(pose.position_m)
          : pose.position_m
        : [0, 0, 0];
    const point = new THREE.Vector3(...position);
    current.device.position.copy(point);
    const orientation =
      orientationValid && pose
        ? poseCoordinates === "robot"
          ? robotToSceneOrientation(pose.orientation_xyzw)
          : new THREE.Quaternion().fromArray(pose.orientation_xyzw).normalize()
        : new THREE.Quaternion();
    current.device.quaternion.copy(orientation);
    current.orientationDevice.quaternion.copy(orientation);
    current.vector.geometry.dispose();
    current.vector.geometry = lineGeometry([new THREE.Vector3(), point]);
    const floor = new THREE.Vector3(point.x, 0, point.z);
    current.projection.geometry.dispose();
    current.projection.geometry = lineGeometry([floor, point]);
    current.projection.computeLineDistances();
    current.floorMarker.position.set(floor.x, 0.004, floor.z);
    current.displacementLabel.textContent = `${active ? "控制中 · " : ""}X ${position[0].toFixed(3)} · Y ${position[1].toFixed(3)} · Z ${position[2].toFixed(3)} m`;
    element.dataset.position = position.join(",");
  }, [active, pose, poseCoordinates]);

  return (
    <div
      ref={mount}
      className={`pose-viewer${className ? ` ${className}` : ""}`}
      aria-label={ariaLabel}
    >
      <div className="pose-viewer-label">
        <span>空间位置 / 自身姿态</span>
        <span
          className="translation-dot"
          data-english="Space / self orientation"
          title="Space / self orientation"
          aria-label="英文：Space / self orientation"
        />
      </div>
      <div className="pose-viewer-toolbar" aria-label="三维视角">
        {cameraViews.map((view) => (
          <button
            key={view.id}
            type="button"
            className="pose-view-button"
            aria-pressed={cameraView === view.id}
            onClick={() => selectCameraView(view.id)}
          >
            {view.label}
          </button>
        ))}
      </div>
      <div className="pose-viewer-inset-label">自身姿态</div>
    </div>
  );
}
