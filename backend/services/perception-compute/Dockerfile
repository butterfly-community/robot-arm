# syntax=docker/dockerfile:1.7
FROM python:3.11-slim-trixie AS build
ENV VIRTUAL_ENV=/opt/venv PATH=/opt/venv/bin:$PATH
RUN apt-get update \
    && apt-get install --yes --no-install-recommends build-essential git \
    && rm -rf /var/lib/apt/lists/*
RUN python -m venv "$VIRTUAL_ENV"
WORKDIR /service
COPY --from=graspgenx_vendor . /opt/graspgenx
COPY backend/patches/graspgenx-device.patch /tmp/graspgenx-device.patch
RUN git -C /opt/graspgenx apply /tmp/graspgenx-device.patch
COPY backend/services/perception-compute/pyproject.toml ./
COPY backend/services/perception-compute/src ./src
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --index-url https://download.pytorch.org/whl/cpu torch==2.6.0+cpu torchvision==0.21.0+cpu \
    && pip install /opt/graspgenx .

FROM python:3.11-slim-trixie
ENV VIRTUAL_ENV=/opt/venv \
    PATH=/opt/venv/bin:$PATH \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    YOLO_CONFIG_DIR=/tmp \
    PERCEPTION_MODEL=/models/yoloe-26s-seg.pt \
    GRASPGENX_CHECKPOINT_DIR=/models/graspgenx \
    GRASPGENX_CHECKPOINT_ROOT=/models/graspgenx/release \
    GRASPGENX_GRIPPER_CFG_DIR=/grippers \
    GRASPGENX_GRIPPER_ASSETS=/grippers
RUN apt-get update \
    && apt-get install --yes --no-install-recommends libgl1 libglib2.0-0 libxcb1 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /service
COPY --from=build /opt/venv /opt/venv
ADD --checksum=sha256:35d7f213e4d75f38514e4656ad3cb91158bd33e3805d8ac349f23b186f66982f \
    https://github.com/ultralytics/assets/releases/download/v8.4.0/mobileclip2_b.ts \
    /service/mobileclip2_b.ts
VOLUME ["/models"]
EXPOSE 8000
CMD ["uvicorn", "perception_compute.app:app", "--app-dir", "/service/src", "--host", "0.0.0.0", "--port", "8000"]
