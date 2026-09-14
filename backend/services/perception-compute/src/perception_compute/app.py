from __future__ import annotations

import base64
import io
import os
import threading
import time
from collections.abc import Callable
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Annotated, Any, Literal, Protocol

import numpy as np
from fastapi import FastAPI, HTTPException
from PIL import Image
from pydantic import BaseModel, Field, model_validator


class TextPrompt(BaseModel):
    kind: Literal["text"] = "text"


class VisualPrompt(BaseModel):
    kind: Literal["visual"] = "visual"
    reference_image_base64: str
    bboxes: list[tuple[float, float, float, float]]
    class_ids: list[int]


SegmentationPrompt = Annotated[TextPrompt | VisualPrompt, Field(discriminator="kind")]


class SegmentRequest(BaseModel):
    image_base64: str
    classes: list[str]
    prompt: SegmentationPrompt = Field(default_factory=TextPrompt)
    model: str | None = None

    @model_validator(mode="after")
    def visual_class_mapping(self):
        if isinstance(self.prompt, VisualPrompt):
            if not self.prompt.bboxes or len(self.prompt.bboxes) != len(self.prompt.class_ids):
                raise ValueError("YOLOE requires one class ID per nonempty visual example box")
            if set(self.prompt.class_ids) != set(range(len(self.classes))):
                raise ValueError("YOLOE visual class IDs must cover the sequential configured classes")
        return self


class Instance(BaseModel):
    instance_id: str
    label: str
    confidence: float
    bounding_box_xyxy: tuple[float, float, float, float]
    mask_width: int
    mask_height: int
    mask_png_base64: str


class SegmentResponse(BaseModel):
    model: str
    image_width: int
    image_height: int
    instances: list[Instance]


class ObservedTcpPose(BaseModel):
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


class ObservedGripper(BaseModel):
    tcp_pose: ObservedTcpPose
    joint_positions_rad: dict[str, float]
    feedback_time_ns: int


class GraspRequest(BaseModel):
    points_xyz_m: list[tuple[float, float, float]]
    scene_points_xyz_m: list[tuple[float, float, float]]
    gripper_asset_id: str
    collision_threshold_m: float = Field(ge=0, allow_inf_nan=False)
    # Null explicitly means no observed robot in this scene (e.g. official fixtures).
    # Production scene-node supplies measured TCP + joints, never commanded targets.
    observed_gripper: ObservedGripper | None


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
    prompt_free: bool

    def segment(self, image: Image.Image, classes: list[str],
                prompt: SegmentationPrompt | None = None) -> list[Instance]: ...


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
    num_grasps: int = 200

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
        self._samplers: dict[str, tuple[Any, np.ndarray, tuple[float, ...]]] = {}
        self._self_filters: dict[str, Any] = {}
        self._lock = threading.Lock()
        # Seed the service's sampling sequence, not every request. Repeated
        # explicit inference must explore new diffusion/environment samples.
        np.random.seed(self.seed)
        torch.manual_seed(self.seed)

    def infer(self, request: GraspRequest) -> GraspResponse:
        import trimesh
        from graspgenx.samplers import run_planner_on_batch
        from perception_compute.grasp_selection import MAX_GRASP_CANDIDATES, representative_grasps
        from perception_compute.gripper_sampling import obb_z_offsets_cm
        from perception_compute.scene_collision import filter_colliding_grasps

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
                offsets = obb_z_offsets_cm(os.path.join(self.gripper_assets, "x_grippers", key))
                self._samplers[key] = sampler, np.asarray(surface, dtype=np.float32), offsets
            sampler, surface, offsets = self._samplers[key]
            started = time.perf_counter()
            [(grasps, scores, branches, _)] = run_planner_on_batch(
                [np.asarray(request.points_xyz_m, dtype=np.float32)],
                sampler,
                # Match demo_scene_pc's default planner and scene-level
                # overrides. Keep diffusion AND top/side OBB candidates;
                # do not rewrite poses or impose a top-only preference.
                planner="graspmoe",
                moe_obb_density="dense-topandside",
                # Official scene-demo offsets only; positive values retreat
                # from the face rather than increasing insertion depth.
                moe_z_offsets_cm=offsets,
                # Preserve scored proposals for complete-plan selection instead
                # of rejecting alternatives solely by the demo's 0.7 cutoff.
                # -1 is the official planner's no-score-cutoff option, not a
                # collision bypass; environment filtering below stays enabled.
                grasp_threshold=-1.0,
                num_grasps=self.num_grasps,
                topk_num_grasps=-1,
            )
            # Official demo_scene_pc pipeline. The environment excludes this
            # target, but retains the support surface and surrounding objects.
            scene = np.asarray(request.scene_points_xyz_m, dtype=np.float32).reshape(-1, 3)
            if request.observed_gripper is not None:
                from perception_compute.gripper_self_filter import GripperSelfFilter
                if key not in self._self_filters:
                    self._self_filters[key] = GripperSelfFilter(
                        os.path.join(self.gripper_assets, "x_grippers", key))
                scene = self._self_filters[key].filter(scene, request.observed_gripper)
            if len(scene) > 8192:  # Official demo's max_scene_points default.
                scene = scene[np.random.choice(len(scene), 8192, replace=False)]
            keep = filter_colliding_grasps(
                scene_pc=scene,
                grasp_poses=grasps,
                gripper_surface_points=surface,
                collision_threshold=request.collision_threshold_m,
            )
            grasps, scores = grasps[keep], scores[keep]
            branches = [branch for branch, valid in zip(branches, keep, strict=True) if valid]
            if len(grasps) > MAX_GRASP_CANDIDATES:
                representatives = representative_grasps(
                    grasps, scores, trimesh.bounds.corners(sampler.gripper.collision_mesh.bounds)
                )
                # Keep original poses, scores and branch provenance together.
                grasps, scores = grasps[representatives], scores[representatives]
                branches = [branches[index] for index in representatives]
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
    prompt_free: bool = False

    def __post_init__(self) -> None:
        from ultralytics import YOLOE

        self._model = YOLOE(self.model_name)
        self._classes: tuple[str, ...] = ()
        self._lock = threading.Lock()

    def segment(self, image: Image.Image, classes: list[str],
                prompt: SegmentationPrompt | None = None) -> list[Instance]:
        if not classes and not self.prompt_free:
            return []
        # FastAPI runs synchronous endpoints concurrently. Prompt mutation and
        # inference must share the lock, not only predict's internal lock.
        with self._lock:
            return self._segment(image, classes, prompt)

    def _segment(self, image: Image.Image, classes: list[str],
                 prompt: SegmentationPrompt | None = None) -> list[Instance]:
        requested = tuple(classes)
        options: dict[str, Any] = {}
        if self.prompt_free:
            if isinstance(prompt, VisualPrompt):
                raise ValueError("免提示词模型不支持视觉提示")
        elif isinstance(prompt, VisualPrompt):
            from ultralytics.models.yolo.yoloe import YOLOEVPSegPredictor

            reference = Image.open(io.BytesIO(base64.b64decode(
                prompt.reference_image_base64, validate=True))).convert("RGB")
            # Official reference-image prompting: boxes refer to this saved
            # image, never to a later camera frame. The model generates masks.
            options = {
                "refer_image": reference,
                "visual_prompts": {"bboxes": np.asarray(prompt.bboxes),
                                   "cls": np.asarray(prompt.class_ids)},
                "predictor": YOLOEVPSegPredictor,
                "imgsz": max(image.size + reference.size),
            }
            self._classes = ()  # Restore text embeddings on the next text request.
        elif requested != self._classes:
            self._model.set_classes(classes)
            self._classes = requested
        if not options and self._model.predictor is not None:
            # A preceding visual request may have used a larger input size.
            # Reset to the library default for the text interface.
            if getattr(self, "_was_visual", False):
                self._model.predictor = None
                self._model.overrides.pop("imgsz", None)
        self._was_visual = isinstance(prompt, VisualPrompt)
        result = self._model.predict(
            image.convert("RGB"),
            device=self.device,
            retina_masks=True,
            verbose=False,
            **options,
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
            # Official visual outputs are object0/object1; class IDs refer
            # to the caller's configured labels, not a fixed model vocabulary.
            label = result.names[class_id] if self.prompt_free else classes[class_id]
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


def default_backends() -> list[ModelBackend]:
    device = os.getenv("PERCEPTION_DEVICE", "cpu")
    return [YoloeBackend(
        model_name=os.getenv("PERCEPTION_MODEL", "yoloe-26x-seg.pt"),
        device=device,
    ), YoloeBackend(
        model_name=os.getenv("PERCEPTION_AUTOMATIC_MODEL", "/models/yoloe-26x-seg-pf.pt"),
        device=device,
        prompt_free=True,
    )]


def default_grasp_backend() -> GraspBackend:
    return GraspGenXBackend(
        checkpoint_root=os.getenv("GRASPGENX_CHECKPOINT_ROOT", "/models/graspgenx/release"),
        gripper_assets=os.getenv("GRASPGENX_GRIPPER_ASSETS", "/grippers"),
        device=os.getenv("GRASPGENX_DEVICE", os.getenv("PERCEPTION_DEVICE", "cpu")),
        seed=int(os.getenv("GRASPGENX_SEED", "0")),
        num_grasps=int(os.getenv("GRASPGENX_NUM_GRASPS", "200")),
    )


def create_app(
    backend_factory: Callable[[], list[ModelBackend]] = default_backends,
    grasp_backend_factory: Callable[[], GraspBackend] = default_grasp_backend,
) -> FastAPI:
    backends: list[ModelBackend] = []
    grasp_backend: GraspBackend | None = None

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        nonlocal backends, grasp_backend
        backends = backend_factory()
        grasp_backend = grasp_backend_factory()
        yield

    app = FastAPI(title="Perception compute service", version="0.1.0", lifespan=lifespan)

    @app.get("/health")
    def health() -> dict[str, str]:
        backend = backends[0]
        return {
            "status": "ready",
            "model": backend.model_name,
            "device": backend.device,
            "grasp_model": grasp_backend.model_name if grasp_backend else "loading",
            "grasp_device": grasp_backend.device if grasp_backend else "loading",
        }

    @app.get("/v1/model")
    def model() -> dict[str, Any]:
        backend = backends[0]
        return {
            "model": backend.model_name,
            "device": backend.device,
            "license": "Ultralytics AGPL-3.0 or Enterprise",
            "models": [{"id": item.model_name,
                        "label": os.path.basename(item.model_name),
                        "prompt_free": item.prompt_free} for item in backends],
        }

    @app.post("/v1/segment")
    def segment(request: SegmentRequest) -> SegmentResponse:
        backend = next((item for item in backends
                        if item.model_name == (request.model or backends[0].model_name)), None)
        if backend is None:
            raise HTTPException(status_code=400, detail="未知分割模型")
        image = Image.open(io.BytesIO(base64.b64decode(request.image_base64, validate=True)))
        try:
            instances = backend.segment(image, request.classes, request.prompt)
        except ValueError as error:
            raise HTTPException(status_code=400, detail=str(error)) from error
        return SegmentResponse(
            model=backend.model_name,
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
