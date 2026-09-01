import asyncio
import base64
import io

from PIL import Image

from perception_compute.app import (
    GraspCandidate,
    GraspRequest,
    GraspResponse,
    Instance,
    SegmentRequest,
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


def encoded_image() -> str:
    image = Image.new("RGB", (8, 6), "red")
    output = io.BytesIO()
    image.save(output, format="PNG")
    return base64.b64encode(output.getvalue()).decode("ascii")


def test_health_and_segmentation_contract() -> None:
    asyncio.run(exercise_contract())


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
                gripper_asset_id="fixture-gripper",
            )
        )
        assert grasp.candidates[0].branch == "fixture-gripper"
        second_grasp = endpoints["/v1/grasps"](
            GraspRequest(
                points_xyz_m=[(0.1, 0.2, 0.3)],
                gripper_asset_id="second-fixture-gripper",
            )
        )
        assert second_grasp.candidates[0].branch == "second-fixture-gripper"
