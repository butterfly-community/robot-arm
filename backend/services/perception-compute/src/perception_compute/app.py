from __future__ import annotations

import base64
import io
import os
from collections.abc import Callable
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Protocol

import numpy as np
from fastapi import FastAPI
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


class ModelBackend(Protocol):
    model_name: str
    device: str

    def segment(self, image: Image.Image, classes: list[str]) -> list[Instance]: ...


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


def create_app(backend_factory: Callable[[], ModelBackend] = default_backend) -> FastAPI:
    backend: ModelBackend | None = None

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        nonlocal backend
        backend = backend_factory()
        yield

    app = FastAPI(title="Perception compute service", version="0.1.0", lifespan=lifespan)

    @app.get("/health")
    def health() -> dict[str, str]:
        assert backend is not None
        return {"status": "ready", "model": backend.model_name, "device": backend.device}

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

    return app


app = create_app()
