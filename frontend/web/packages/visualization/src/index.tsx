"use client";

import type { AbsolutePoseFrame } from "@robot/contracts";
import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

type Vector3Tuple = [number, number, number];
type QuaternionTuple = [number, number, number, number];

export type PoseVisualization = {
  position_m: Vector3Tuple;
  orientation_xyzw: QuaternionTuple;
};

type Props = {
  pose?: AbsolutePoseFrame | PoseVisualization;
  originM?: Vector3Tuple;
  axes?: number[][];
  translationScale?: number;
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

function createDeviceModel() {
  const group = new THREE.Group();
  const shell = new THREE.MeshStandardMaterial({
    color: 0xd8e1e8,
    metalness: 0.2,
    roughness: 0.38,
  });
  const dark = new THREE.MeshStandardMaterial({
    color: 0x111820,
    metalness: 0.1,
    roughness: 0.55,
  });
  const cyan = new THREE.MeshStandardMaterial({
    color: 0x43d9e6,
    emissive: 0x123d43,
    emissiveIntensity: 1.4,
    roughness: 0.32,
  });
  const handle = new THREE.Mesh(
    new THREE.CylinderGeometry(0.024, 0.032, 0.14, 28),
    shell,
  );
  handle.rotation.x = Math.PI / 2;
  handle.position.z = 0.025;
  group.add(handle);
  const head = new THREE.Mesh(new THREE.SphereGeometry(0.042, 32, 20), shell);
  head.scale.set(1, 0.72, 1.18);
  head.position.z = -0.065;
  group.add(head);
  const pad = new THREE.Mesh(
    new THREE.CylinderGeometry(0.021, 0.021, 0.007, 32),
    dark,
  );
  pad.position.set(0, 0.034, -0.038);
  group.add(pad);
  const front = new THREE.Mesh(new THREE.ConeGeometry(0.012, 0.035, 24), cyan);
  front.rotation.x = -Math.PI / 2;
  front.position.z = -0.118;
  group.add(front);
  const top = new THREE.Mesh(new THREE.ConeGeometry(0.008, 0.025, 20), cyan);
  top.position.set(0, 0.066, -0.035);
  group.add(top);
  return group;
}

function mappedPosition(
  position: Vector3Tuple,
  origin: Vector3Tuple,
  axes: number[][],
  scale: number,
): Vector3Tuple {
  const delta = position.map((value, index) => value - origin[index]);
  if (axes.length !== 3 || axes.some((row) => row.length !== 3)) {
    return delta as Vector3Tuple;
  }
  return axes.map(
    (row) =>
      row.reduce((sum, value, index) => sum + value * delta[index], 0) * scale,
  ) as Vector3Tuple;
}

function mappedOrientation(orientation: QuaternionTuple, axes: number[][]) {
  const source = new THREE.Quaternion().fromArray(orientation).normalize();
  if (axes.length !== 3 || axes.some((row) => row.length !== 3)) {
    return source;
  }
  const basis = new THREE.Matrix4().set(
    axes[0][0],
    axes[0][1],
    axes[0][2],
    0,
    axes[1][0],
    axes[1][1],
    axes[1][2],
    0,
    axes[2][0],
    axes[2][1],
    axes[2][2],
    0,
    0,
    0,
    0,
    1,
  );
  return new THREE.Quaternion()
    .setFromRotationMatrix(basis)
    .multiply(source)
    .normalize();
}

function lineGeometry(points: THREE.Vector3[]) {
  return new THREE.BufferGeometry().setFromPoints(points);
}

export function PoseViewer({
  pose,
  originM = [0, 0, 0],
  axes = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ],
  translationScale = 1,
  active = false,
  className,
  ariaLabel = "三维空间位置和设备自身姿态",
}: Props) {
  const mount = useRef<HTMLDivElement>(null);
  const state = useRef<SceneState | undefined>(undefined);

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
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    element.appendChild(renderer.domElement);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.set(0, 0.08, 0);
    controls.update();

    scene.add(new THREE.HemisphereLight(0xbdeeff, 0x111820, 2.2));
    const key = new THREE.DirectionalLight(0xffffff, 3.4);
    key.position.set(0.7, 1.1, 0.8);
    scene.add(key);
    const rim = new THREE.PointLight(0x43d9e6, 8, 3);
    rim.position.set(-0.8, 0.45, -0.7);
    scene.add(rim);

    const grid = new THREE.GridHelper(2, 20, 0x315761, 0x16242b);
    scene.add(grid);
    const axesHelper = new THREE.AxesHelper(0.22);
    axesHelper.position.y = 0.004;
    scene.add(axesHelper);
    const origin = new THREE.Mesh(
      new THREE.RingGeometry(0.035, 0.052, 40),
      new THREE.MeshBasicMaterial({ color: 0x43d9e6, side: THREE.DoubleSide }),
    );
    origin.rotation.x = -Math.PI / 2;
    origin.position.y = 0.003;
    scene.add(origin);

    const device = createDeviceModel();
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
    orientationScene.add(new THREE.AxesHelper(0.12));
    const orientationDevice = createDeviceModel();
    orientationDevice.scale.setScalar(1.35);
    orientationScene.add(orientationDevice);
    const orientationCamera = new THREE.PerspectiveCamera(34, 1, 0.01, 5);
    orientationCamera.position.set(0.3, 0.22, 0.38);
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
        materials.forEach((material) => material.dispose());
      });
      orientationScene.traverse((object) => {
        if (!(object instanceof THREE.Mesh || object instanceof THREE.Line))
          return;
        object.geometry.dispose();
        const materials = Array.isArray(object.material)
          ? object.material
          : [object.material];
        materials.forEach((material) => material.dispose());
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
    const valid =
      pose != null &&
      pose.position_m.every(Number.isFinite) &&
      pose.orientation_xyzw.every(Number.isFinite);
    current.device.visible = valid;
    current.orientationDevice.visible = valid;
    current.vector.visible = valid;
    current.projection.visible = valid;
    current.floorMarker.visible = valid;
    element.dataset.poseReady = String(valid);
    element.dataset.controlActive = String(active);
    if (!valid || !pose) {
      current.displacementLabel.textContent = "等待有效位姿";
      return;
    }
    const position = mappedPosition(
      pose.position_m,
      originM,
      axes,
      translationScale,
    );
    const point = new THREE.Vector3(...position);
    const viewDelta = point.clone().sub(current.controls.target);
    current.camera.position.add(viewDelta);
    current.controls.target.copy(point);
    current.controls.update();
    current.device.position.copy(point);
    const orientation = mappedOrientation(pose.orientation_xyzw, axes);
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
  }, [active, axes, originM, pose, translationScale]);

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
      <div className="pose-viewer-inset-label">自身姿态</div>
    </div>
  );
}
