# syntax=docker/dockerfile:1.7
FROM python:3.13-slim-trixie

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    YOLO_CONFIG_DIR=/tmp \
    PERCEPTION_MODEL=/models/yoloe-26s-seg.pt

WORKDIR /service
RUN apt-get update \
    && apt-get install --yes --no-install-recommends libgl1 libglib2.0-0 libxcb1 \
    && rm -rf /var/lib/apt/lists/*
COPY pyproject.toml ./
COPY src ./src
ADD --checksum=sha256:35d7f213e4d75f38514e4656ad3cb91158bd33e3805d8ac349f23b186f66982f \
    https://github.com/ultralytics/assets/releases/download/v8.4.0/mobileclip2_b.ts \
    /service/mobileclip2_b.ts
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --index-url https://download.pytorch.org/whl/cpu torch torchvision \
    && pip install .

VOLUME ["/models"]
EXPOSE 8000
CMD ["uvicorn", "perception_compute.app:app", "--app-dir", "/service/src", "--host", "0.0.0.0", "--port", "8000"]
