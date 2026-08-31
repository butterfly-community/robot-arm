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
from pydantic import BaseModel


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
    gripper_name: str
    planner: str = "graspmoe"


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
        from graspgenx.grasp_server import GraspGenXSampler, load_grasp_gen_model
        from graspgenx.utils.checkpoint_io import load_model_cfg

        self._sampler_type = GraspGenXSampler
        self._config = load_model_cfg(
            os.path.join(self.checkpoint_root, "gen"),
            os.path.join(self.checkpoint_root, "dis"),
        )
        self._model = load_grasp_gen_model(self._config, device=self.device)
        self._samplers: dict[str, Any] = {}
        self._lock = threading.Lock()

    def infer(self, request: GraspRequest) -> GraspResponse:
        import torch
        from graspgenx.samplers import run_planner_on_object

        key = request.gripper_name
        with self._lock:
            np.random.seed(self.seed)
            torch.manual_seed(self.seed)
            sampler = self._samplers.get(key)
            if sampler is None:
                sampler = self._sampler_type(
                    self._config,
                    gripper_name=request.gripper_name,
                    assets_dir=self.gripper_assets,
                    model=self._model,
                )
                self._samplers[key] = sampler
            started = time.perf_counter()
            grasps, scores, branches, _ = run_planner_on_object(
                np.asarray(request.points_xyz_m, dtype=np.float32),
                sampler,
                planner=request.planner,
                grasp_threshold=-1.0,
                topk_num_grasps=-1,
            )
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
        requested = tuple(classes)
        if requested != self._classes:
            self._model.set_classes(classes)
            self._classes = requested
        result = self._model.predict(
            np.asarray(image.convert("RGB")), device=self.device, verbose=False
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
            mask_image = Image.fromarray((mask * 255).astype(np.uint8)).resize(
                image.size, Image.Resampling.NEAREST
            )
            encoded = io.BytesIO()
            mask_image.save(encoded, format="PNG")
            instances.append(
                Instance(
                    instance_id=f"{classes[class_id]}-{index}",
                    label=classes[class_id],
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
        model_name=os.getenv("PERCEPTION_MODEL", "yoloe-26s-seg.pt"),
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
