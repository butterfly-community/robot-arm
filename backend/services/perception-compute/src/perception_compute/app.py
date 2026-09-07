from __future__ import annotations

import base64
import io
import os
import threading
import time
from collections.abc import Callable
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np
from fastapi import FastAPI, HTTPException
from PIL import Image
from pydantic import BaseModel, Field


class SegmentRequest(BaseModel):
    image_base64: str
    classes: list[str]


class Instance(BaseModel):
    instance_id: str
    label: str
    confidence: float
    bounding_box_xyxy: tuple[float, float, float, float]
    mask_width: int
    mask_height: int
    mask_png_base64: str


class SegmentResponse(BaseModel):
    image_width: int
    image_height: int
    instances: list[Instance]


class GraspRequest(BaseModel):
    points_xyz_m: list[tuple[float, float, float]]
    scene_points_xyz_m: list[tuple[float, float, float]]
    gripper_asset_id: str
    collision_threshold_m: float = Field(ge=0, allow_inf_nan=False)


class GraspCandidate(BaseModel):
    transform: tuple[
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
    ]
    confidence: float
    branch: str


class GraspResponse(BaseModel):
    candidates: list[GraspCandidate]
    inference_ms: float


class ModelBackend(Protocol):
    model_name: str
    device: str

    def segment(self, image: Image.Image, classes: list[str]) -> list[Instance]: ...


class GraspBackend(Protocol):
    model_name: str
    device: str

    def infer(self, request: GraspRequest) -> GraspResponse: ...


@dataclass
class GraspGenXBackend:
    checkpoint_root: str
    gripper_assets: str
    device: str
    seed: int

    model_name = "GraspGenX"

    def __post_init__(self) -> None:
        import torch
        from graspgenx.grasp_server import GraspGenXSampler, load_grasp_gen_model
        from graspgenx.utils.checkpoint_io import load_model_cfg

        self._sampler_type = GraspGenXSampler
        self._config = load_model_cfg(
            os.path.join(self.checkpoint_root, "gen"),
            os.path.join(self.checkpoint_root, "dis"),
        )
        self._model = load_grasp_gen_model(self._config, device=self.device)
        self._samplers: dict[str, tuple[Any, np.ndarray]] = {}
        self._lock = threading.Lock()
        # Seed the service's sampling sequence, not every request. Repeated
        # explicit inference must explore new diffusion/environment samples.
        np.random.seed(self.seed)
        torch.manual_seed(self.seed)

    def infer(self, request: GraspRequest) -> GraspResponse:
        import trimesh
        from graspgenx.samplers import run_planner_on_batch
        from graspgenx.utils.collision_filter import filter_colliding_grasps

        key = request.gripper_asset_id
        with self._lock:
            if key not in self._samplers:
                sampler = self._sampler_type(
                    self._config,
                    gripper_name=request.gripper_asset_id,
                    assets_dir=self.gripper_assets,
                    model=self._model,
                )
                # demo_scene_pc samples the open mesh once per gripper and
                # reuses those points across objects and scene requests.
                surface, _ = trimesh.sample.sample_surface(sampler.gripper.collision_mesh, 2000)
                self._samplers[key] = sampler, np.asarray(surface, dtype=np.float32)
            sampler, surface = self._samplers[key]
            started = time.perf_counter()
            [(grasps, scores, branches, _)] = run_planner_on_batch(
                [np.asarray(request.points_xyz_m, dtype=np.float32)],
                sampler,
                # Match demo_scene_pc's default planner and scene-level
                # overrides. Keep diffusion AND top/side OBB candidates;
                # do not rewrite poses or impose a top-only preference.
                planner="graspmoe",
                moe_obb_density="dense-topandside",
                moe_z_offsets_cm=(-2, 0),
                # Match demo_scene_pc.py, not the lower-level library's -1
                # default (which also returns rejected, low-quality grasps).
                grasp_threshold=0.7,
                num_grasps=200,
                topk_num_grasps=-1,
            )
            # Official demo_scene_pc pipeline. The environment excludes this
            # target, but retains the support surface and surrounding objects.
            scene = np.asarray(request.scene_points_xyz_m, dtype=np.float32).reshape(-1, 3)
            if len(scene) > 8192:  # Official demo's max_scene_points default.
                scene = scene[np.random.choice(len(scene), 8192, replace=False)]
            keep = filter_colliding_grasps(
                scene_pc=scene,
                grasp_poses=grasps,
                gripper_surface_points=surface,
                collision_threshold=request.collision_threshold_m,
                device=self.device,
            )
            grasps, scores = grasps[keep], scores[keep]
            branches = [branch for branch, valid in zip(branches, keep, strict=True) if valid]
            inference_ms = (time.perf_counter() - started) * 1000.0
        return GraspResponse(
            candidates=[
                GraspCandidate(
                    # GraspGenX predicts its canonical gripper-base pose. The
                    # descriptor owns the base -> real TCP transform, so the
                    # service boundary always returns the requested gripper's
                    # TCP pose in the input point-cloud frame.
                    transform=tuple(
                        tuple(float(value) for value in row)
                        for row in grasp @ sampler.gripper.tool_tcp_transform
                    ),
                    confidence=float(score),
                    branch=str(branch),
                )
                for grasp, score, branch in zip(grasps, scores, branches, strict=True)
            ],
            inference_ms=inference_ms,
        )


@dataclass
class YoloeBackend:
    model_name: str
    device: str

    def __post_init__(self) -> None:
        from ultralytics import YOLOE

        self._model = YOLOE(self.model_name)
        self._classes: tuple[str, ...] = ()

    def segment(self, image: Image.Image, classes: list[str]) -> list[Instance]:
        if not classes:
            return []
        requested = tuple(classes)
        if requested != self._classes:
            self._model.set_classes(classes)
            self._classes = requested
        result = self._model.predict(
            image.convert("RGB"),
            device=self.device,
            retina_masks=True,
            verbose=False,
        )[0]
        if result.boxes is None or result.masks is None:
            return []
        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy().astype(int)
        masks = result.masks.data.cpu().numpy()
        instances = []
        for index, (box, score, class_id, mask) in enumerate(
            zip(boxes, scores, class_ids, masks, strict=True)
        ):
            label = result.names[class_id]
            mask_image = Image.fromarray((mask * 255).astype(np.uint8))
            encoded = io.BytesIO()
            mask_image.save(encoded, format="PNG")
            instances.append(
                Instance(
                    instance_id=f"{label}-{index}",
                    label=label,
                    confidence=float(score),
                    bounding_box_xyxy=tuple(float(value) for value in box),
                    mask_width=image.width,
                    mask_height=image.height,
                    mask_png_base64=base64.b64encode(encoded.getvalue()).decode("ascii"),
                )
            )
        return instances


def default_backend() -> ModelBackend:
    return YoloeBackend(
        model_name=os.getenv("PERCEPTION_MODEL", "yoloe-26x-seg.pt"),
        device=os.getenv("PERCEPTION_DEVICE", "cpu"),
    )


def default_grasp_backend() -> GraspBackend:
    return GraspGenXBackend(
        checkpoint_root=os.getenv("GRASPGENX_CHECKPOINT_ROOT", "/models/graspgenx/release"),
        gripper_assets=os.getenv("GRASPGENX_GRIPPER_ASSETS", "/grippers"),
        device=os.getenv("GRASPGENX_DEVICE", os.getenv("PERCEPTION_DEVICE", "cpu")),
        seed=int(os.getenv("GRASPGENX_SEED", "0")),
    )


def create_app(
    backend_factory: Callable[[], ModelBackend] = default_backend,
    grasp_backend_factory: Callable[[], GraspBackend] = default_grasp_backend,
) -> FastAPI:
    backend: ModelBackend | None = None
    grasp_backend: GraspBackend | None = None

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        nonlocal backend, grasp_backend
        backend = backend_factory()
        grasp_backend = grasp_backend_factory()
        yield

    app = FastAPI(title="Perception compute service", version="0.1.0", lifespan=lifespan)

    @app.get("/health")
    def health() -> dict[str, str]:
        assert backend is not None
        return {
            "status": "ready",
            "model": backend.model_name,
            "device": backend.device,
            "grasp_model": grasp_backend.model_name if grasp_backend else "loading",
            "grasp_device": grasp_backend.device if grasp_backend else "loading",
        }

    @app.get("/v1/model")
    def model() -> dict[str, str]:
        assert backend is not None
        return {
            "model": backend.model_name,
            "device": backend.device,
            "license": "Ultralytics AGPL-3.0 or Enterprise",
        }

    @app.post("/v1/segment")
    def segment(request: SegmentRequest) -> SegmentResponse:
        assert backend is not None
        image = Image.open(io.BytesIO(base64.b64decode(request.image_base64, validate=True)))
        instances = backend.segment(image, request.classes)
        return SegmentResponse(
            image_width=image.width,
            image_height=image.height,
            instances=instances,
        )

    @app.post("/v1/grasps")
    def grasps(request: GraspRequest) -> GraspResponse:
        assert grasp_backend is not None
        try:
            return grasp_backend.infer(request)
        except Exception as error:
            raise HTTPException(status_code=502, detail=str(error)) from error

    return app


app = create_app()
