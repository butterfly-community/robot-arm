import base64
import io

from fastapi.testclient import TestClient
from PIL import Image

from perception_compute.app import Instance, create_app


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


def encoded_image() -> str:
    image = Image.new("RGB", (8, 6), "red")
    output = io.BytesIO()
    image.save(output, format="PNG")
    return base64.b64encode(output.getvalue()).decode("ascii")


def test_health_and_segmentation_contract() -> None:
    with TestClient(create_app(FakeBackend)) as client:
        assert client.get("/health").json() == {
            "status": "ready",
            "model": "fixture-seg",
            "device": "cpu",
        }
        response = client.post(
            "/v1/segment",
            json={
                "image_base64": encoded_image(),
                "classes": ["red cube", "gray storage bin"],
            },
        )
        response.raise_for_status()
        payload = response.json()
        assert payload["image_width"] == 8
        assert payload["image_height"] == 6
        assert payload["instances"][0]["label"] == "red cube"
        assert Image.open(
            io.BytesIO(base64.b64decode(payload["instances"][0]["mask_png_base64"]))
        ).size == (8, 6)
