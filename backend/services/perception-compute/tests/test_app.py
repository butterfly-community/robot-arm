import asyncio
import base64
import io
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import numpy as np
from PIL import Image

from perception_compute.app import (
    GraspCandidate,
    GraspGenXBackend,
    GraspRequest,
    GraspResponse,
    Instance,
    SegmentRequest,
    YoloeBackend,
    create_app,
)


class FakeBackend:
    model_name = "fixture-seg"
    device = "cpu"

    def segment(self, image: Image.Image, classes: list[str]) -> list[Instance]:
        mask = Image.new("L", image.size, 255)
        encoded = io.BytesIO()
        mask.save(encoded, format="PNG")
        return [
            Instance(
                instance_id="red-cube-0",
                label=classes[0],
                confidence=1.0,
                bounding_box_xyxy=(0.0, 0.0, float(image.width), float(image.height)),
                mask_width=image.width,
                mask_height=image.height,
                mask_png_base64=base64.b64encode(encoded.getvalue()).decode("ascii"),
            )
        ]


class FakeGraspBackend:
    model_name = "GraspGenX-fixture"
    device = "cpu"

    def infer(self, request: GraspRequest) -> GraspResponse:
        assert request.points_xyz_m == [(0.1, 0.2, 0.3)]
        return GraspResponse(
            candidates=[
                GraspCandidate(
                    transform=(
                        (1.0, 0.0, 0.0, 0.1),
                        (0.0, 1.0, 0.0, 0.2),
                        (0.0, 0.0, 1.0, 0.3),
                        (0.0, 0.0, 0.0, 1.0),
                    ),
                    confidence=0.9,
                    branch=request.gripper_asset_id,
                )
            ],
            inference_ms=12.0,
        )


class FakeTensor:
    def __init__(self, values: list | np.ndarray) -> None:
        self.values = np.asarray(values)

    def cpu(self) -> "FakeTensor":
        return self

    def numpy(self) -> np.ndarray:
        return self.values


class FakeYoloeModel:
    def __init__(self) -> None:
        self.classes: list[str] = []
        self.predict_calls = 0
        self.predict_options: list[dict] = []
        self.last_image: Image.Image | None = None

    def set_classes(self, classes: list[str]) -> None:
        self.classes = classes

    def predict(self, image: Image.Image, **options):
        assert isinstance(image, Image.Image)
        assert image.mode == "RGB"
        self.last_image = image.copy()
        assert options["retina_masks"] is True
        self.predict_options.append(options)
        assert "conf" not in options
        self.predict_calls += 1
        boxes = SimpleNamespace(
            xyxy=FakeTensor([[1.0, 1.0, 3.0, 3.0]]),
            conf=FakeTensor([0.9]),
            cls=FakeTensor([0]),
        )
        masks = SimpleNamespace(data=FakeTensor([np.ones((6, 8))]))
        return [SimpleNamespace(boxes=boxes, masks=masks, names={0: self.classes[0]})]


def encoded_image() -> str:
    image = Image.new("RGB", (8, 6), "red")
    output = io.BytesIO()
    image.save(output, format="PNG")
    return base64.b64encode(output.getvalue()).decode("ascii")


def test_health_and_segmentation_contract() -> None:
    asyncio.run(exercise_contract())


def test_yoloe_uses_official_image_and_native_mask_contract() -> None:
    backend = YoloeBackend.__new__(YoloeBackend)
    backend.model_name = "fixture-seg"
    backend.device = "cpu"
    backend._model = FakeYoloeModel()
    backend._classes = ()
    backend._lock = threading.Lock()
    image = Image.new("RGB", (8, 6), "red")

    assert backend.segment(image, []) == []
    assert backend._model.predict_calls == 0

    instances = backend.segment(image, ["red cube"])
    assert backend._model.classes == ["red cube"]
    assert "imgsz" not in backend._model.predict_options[-1]
    assert backend._model.last_image.getpixel((0, 0)) == (255, 0, 0)
    assert instances[0].label == "red cube"
    mask = Image.open(io.BytesIO(base64.b64decode(instances[0].mask_png_base64)))
    assert mask.size == image.size


def test_official_yoloe_loader_and_predictor_preserve_rgb() -> None:
    """PIL is RGB; Ultralytics owns its internal BGR loader convention."""
    import torch
    from ultralytics.data.loaders import LoadPilAndNumpy
    from ultralytics.engine.predictor import BasePredictor

    rgb = np.zeros((32, 32, 3), dtype=np.uint8)
    rgb[:16, :16] = [255, 0, 0]
    rgb[:16, 16:] = [0, 255, 0]
    rgb[16:, :16] = [0, 0, 255]
    rgb[16:, 16:] = [17, 83, 201]
    loader = LoadPilAndNumpy(Image.fromarray(rgb))
    _, images, _ = next(iter(loader))
    np.testing.assert_array_equal(images[0], rgb[..., ::-1])

    predictor = BasePredictor.__new__(BasePredictor)
    predictor.device = torch.device("cpu")
    predictor.model = SimpleNamespace(fp16=False, stride=32)
    predictor.args = SimpleNamespace(rect=False)
    predictor.imgsz = [32, 32]
    actual = predictor.preprocess(images)[0].permute(1, 2, 0).numpy()
    np.testing.assert_array_equal(actual, rgb.astype(np.float32) / 255)


def test_yoloe_keeps_source_resolution_without_overriding_model_input_size() -> None:
    backend = YoloeBackend.__new__(YoloeBackend)
    backend.model_name = "fixture-seg"
    backend.device = "cpu"
    backend._model = FakeYoloeModel()
    backend._classes = ()
    backend._lock = threading.Lock()

    for size in ((1280, 720), (1920, 1080)):
        backend.segment(Image.new("RGB", size), ["object"])
        assert backend._model.last_image.size == size
        assert "imgsz" not in backend._model.predict_options[-1]


def test_prompt_update_and_inference_are_one_serial_operation() -> None:
    import time

    backend = YoloeBackend.__new__(YoloeBackend)
    backend._lock = threading.Lock()
    active = 0
    maximum_active = 0

    def segment(image, classes):
        nonlocal active, maximum_active
        active += 1
        maximum_active = max(maximum_active, active)
        time.sleep(0.01)
        active -= 1
        return classes

    backend._segment = segment
    with ThreadPoolExecutor(max_workers=4) as executor:
        results = list(
            executor.map(
                lambda label: backend.segment(Image.new("RGB", (1, 1)), [label]),
                ["cube", "bin", "apple", "bottle"],
            )
        )
    assert results == [["cube"], ["bin"], ["apple"], ["bottle"]]
    assert maximum_active == 1


def test_graspgenx_scene_workflow_filters_base_poses_before_tcp_conversion(monkeypatch):
    import trimesh

    backend = GraspGenXBackend.__new__(GraspGenXBackend)
    backend.seed = 0
    backend.device = "cpu"
    backend._lock = threading.Lock()
    tool = np.eye(4)
    tool[2, 3] = 0.09
    mesh = object()
    surface = np.zeros((2000, 3), dtype=np.float32)
    sampler = SimpleNamespace(gripper=SimpleNamespace(tool_tcp_transform=tool, collision_mesh=mesh))
    backend._samplers = {}
    backend._config = object()
    backend._model = object()
    backend.gripper_assets = "test-assets"
    backend._sampler_type = lambda *args, **kwargs: sampler
    surface_calls = []

    def sample_mesh(actual_mesh, count):
        assert actual_mesh is mesh
        assert count == 2000
        surface_calls.append(count)
        return surface, None

    monkeypatch.setattr(trimesh.sample, "sample_surface", sample_mesh)
    poses = np.array([np.eye(4)] * 3)
    # Distinct approach axes must survive unchanged; TCP conversion is in the
    # gripper frame, not a world-Z correction or a top-down orientation rewrite.
    poses[2, :3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]

    def unexpected_reseed(*args, **kwargs):
        raise AssertionError("inference must advance sampling, not restart its seed")

    monkeypatch.setattr(np.random, "seed", unexpected_reseed)

    def run(points, actual_sampler, **options):
        np.testing.assert_allclose(points, [[[0.1, 0.2, 0.3]]])
        assert actual_sampler is sampler
        assert options == {
            "planner": "graspmoe",
            "moe_obb_density": "dense-topandside",
            "moe_z_offsets_cm": (-2, 0),
            "grasp_threshold": 0.7,
            "num_grasps": 200,
            "topk_num_grasps": -1,
        }
        return [
            (poses, np.array([0.8, 0.7, 0.9]), ["diff", "obb", "obb"], None)
        ]

    def filter_scene(**options):
        np.testing.assert_allclose(options["scene_pc"], [[0.0, 0.0, 0.0]])
        assert options["gripper_surface_points"] is surface
        assert options["collision_threshold"] == 0.01
        assert options["device"] == "cpu"
        np.testing.assert_allclose(options["grasp_poses"], poses)
        assert len(options["grasp_poses"]) == 3
        return np.array([True, False, True])

    monkeypatch.setitem(
        sys.modules, "graspgenx.samplers", SimpleNamespace(run_planner_on_batch=run)
    )
    monkeypatch.setitem(
        sys.modules,
        "graspgenx.utils.collision_filter",
        SimpleNamespace(filter_colliding_grasps=filter_scene),
    )
    result = backend.infer(
        GraspRequest(
            points_xyz_m=[(0.1, 0.2, 0.3)],
            scene_points_xyz_m=[(0.0, 0.0, 0.0)],
            gripper_asset_id="test-tool",
            collision_threshold_m=0.01,
        )
    )
    assert len(result.candidates) == 2
    np.testing.assert_allclose(result.candidates[0].transform, tool)
    assert result.candidates[0].branch == "diff"
    np.testing.assert_allclose(result.candidates[1].transform, poses[2] @ tool)
    assert result.candidates[1].branch == "obb"
    backend.infer(
        GraspRequest(
            points_xyz_m=[(0.1, 0.2, 0.3)],
            scene_points_xyz_m=[(0.0, 0.0, 0.0)],
            gripper_asset_id="test-tool",
            collision_threshold_m=0.01,
        )
    )
    assert surface_calls == [2000]


async def exercise_contract() -> None:
    app = create_app(FakeBackend, FakeGraspBackend)
    async with app.router.lifespan_context(app):
        endpoints = {
            route.path: route.endpoint for route in app.routes if hasattr(route, "endpoint")
        }
        assert endpoints["/health"]() == {
            "status": "ready",
            "model": "fixture-seg",
            "device": "cpu",
            "grasp_model": "GraspGenX-fixture",
            "grasp_device": "cpu",
        }
        response = endpoints["/v1/segment"](
            SegmentRequest(
                image_base64=encoded_image(),
                classes=["red cube", "gray storage bin"],
            )
        )
        assert response.image_width == 8
        assert response.image_height == 6
        assert response.instances[0].label == "red cube"
        assert Image.open(
            io.BytesIO(base64.b64decode(response.instances[0].mask_png_base64))
        ).size == (8, 6)
        grasp = endpoints["/v1/grasps"](
            GraspRequest(
                points_xyz_m=[(0.1, 0.2, 0.3)],
                scene_points_xyz_m=[(0.0, 0.0, 0.0)],
                gripper_asset_id="fixture-gripper",
                collision_threshold_m=0.01,
            )
        )
        assert grasp.candidates[0].branch == "fixture-gripper"
        second_grasp = endpoints["/v1/grasps"](
            GraspRequest(
                points_xyz_m=[(0.1, 0.2, 0.3)],
                scene_points_xyz_m=[(0.0, 0.0, 0.0)],
                gripper_asset_id="second-fixture-gripper",
                collision_threshold_m=0.01,
            )
        )
        assert second_grasp.candidates[0].branch == "second-fixture-gripper"
