# syntax=docker/dockerfile:1.7
FROM ubuntu:26.04

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# Common command-line tools
RUN apt-get update \
    && apt-get install --yes ca-certificates curl git gnupg

# Native backend build and USB dependencies
RUN apt-get update \
    && apt-get install --yes \
       build-essential cmake libudev-dev libusb-1.0-0-dev pkg-config

# ROS 2 Lyrical package repository
RUN curl -fsSL -o /tmp/ros2-apt-source.deb \
       https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.resolute_all.deb \
    && echo "a275b9b819874e745a928e83e39c429fa4d607159285c4ef3ebcf75afa732ee3  /tmp/ros2-apt-source.deb" \
       | sha256sum --check --strict \
    && apt-get install --yes /tmp/ros2-apt-source.deb

# ROS 2 core and development tools
RUN apt-get update \
    && apt-get install --yes \
       ros-dev-tools \
       ros-lyrical-joint-state-topic-hardware-interface \
       ros-lyrical-ros-base \
    && rosdep init \
    && rosdep update --rosdistro lyrical
ENV ROS_DISTRO=lyrical

# MoveIt 2, MTC and kinematics dependencies
RUN apt-get update \
    && apt-get install --yes \
       ros-lyrical-moveit \
       ros-lyrical-moveit-task-constructor-capabilities \
       ros-lyrical-moveit-task-constructor-core \
       ros-lyrical-moveit-task-constructor-msgs \
       ros-lyrical-moveit-task-constructor-visualization \
       ros-lyrical-trac-ik-kinematics-plugin

# Patched MoveIt Servo
ARG MOVEIT_REPOSITORY=https://github.com/moveit/moveit2.git
ARG MOVEIT_REF=2.15.0
COPY backend/patches/moveit-servo-halt-pose-at-joint-bound.patch /tmp/moveit-servo.patch
RUN git clone --depth=1 --branch "$MOVEIT_REF" "$MOVEIT_REPOSITORY" /opt/moveit_ws/src/moveit2 \
    && git -C /opt/moveit_ws/src/moveit2 apply --ignore-space-change --ignore-whitespace \
       /tmp/moveit-servo.patch \
    && source /opt/ros/lyrical/setup.bash \
    && cd /opt/moveit_ws \
    && colcon build --packages-select moveit_servo --cmake-args -DBUILD_TESTING=OFF

# Browser VNC for RViz
RUN apt-get update \
    && apt-get install --yes \
       fonts-noto-cjk novnc openbox tigervnc-standalone-server websockify

# RealSense SDK package repository and native driver
RUN install -d /etc/apt/keyrings \
    && curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
       | gpg --dearmor -o /etc/apt/keyrings/librealsenseai.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo resolute main" \
       > /etc/apt/sources.list.d/librealsense.list \
    && apt-get update \
    && apt-get install --yes librealsense2-dev librealsense2-utils

# OpenCV 5 native dependencies
RUN apt-get update \
    && apt-get install --yes \
       clang g++-16 libclang-dev libgl1 libglib2.0-0 libxcb1 ninja-build

# OpenCV 5 source build
ENV OpenCV_DIR=/opt/opencv5/lib/cmake/opencv5 \
    LD_LIBRARY_PATH=/opt/opencv5/lib \
    PKG_CONFIG_PATH=/opt/opencv5/lib/pkgconfig
RUN git clone --depth=1 --branch 5.0.0 https://github.com/opencv/opencv.git /opencv \
    && cmake -S /opencv -B /opencv-build -G Ninja \
       -DCMAKE_BUILD_TYPE=Release \
       -DCMAKE_INSTALL_PREFIX=/opt/opencv5 \
       -DBUILD_LIST=core,calib,geometry,imgcodecs,imgproc,objdetect \
       -DBUILD_opencv_apps=OFF \
       -DBUILD_PERF_TESTS=OFF \
       -DBUILD_TESTS=OFF \
       -DBUILD_EXAMPLES=OFF \
    && cmake --build /opencv-build \
    && cmake --install /opencv-build

# ROS image bridge built against the same OpenCV 5 installation
ARG VISION_OPENCV_REPOSITORY=https://github.com/ros-perception/vision_opencv.git
ARG VISION_OPENCV_REF=f5b738d9694f0cee5904440d03912fc249943f8a
COPY backend/patches/cv-bridge-opencv5.patch /tmp/cv-bridge-opencv5.patch
RUN git clone "$VISION_OPENCV_REPOSITORY" /opt/cv_bridge_ws/src/vision_opencv \
    && git -C /opt/cv_bridge_ws/src/vision_opencv checkout "$VISION_OPENCV_REF" \
    && git -C /opt/cv_bridge_ws/src/vision_opencv apply /tmp/cv-bridge-opencv5.patch \
    && source /opt/ros/lyrical/setup.bash \
    && cd /opt/cv_bridge_ws \
    && colcon build --packages-select cv_bridge \
       --cmake-args -DBUILD_TESTING=OFF -DCV_BRIDGE_DISABLE_PYTHON=ON

# RealSense ROS camera driver
COPY backend/patches/realsense-ros-lyrical.patch /tmp/realsense-ros-lyrical.patch
RUN git clone --depth=1 --branch 4.58.3 \
       https://github.com/realsenseai/realsense-ros.git /opt/realsense_ws/src/realsense-ros \
    && git -C /opt/realsense_ws/src/realsense-ros apply /tmp/realsense-ros-lyrical.patch \
    && source /opt/ros/lyrical/setup.bash \
    && source /opt/cv_bridge_ws/install/setup.bash \
    && cd /opt/realsense_ws \
    && rosdep install --from-paths src --ignore-src --rosdistro lyrical -y \
       --dependency-types build --dependency-types build_export \
       --dependency-types buildtool --dependency-types buildtool_export \
       --dependency-types exec \
       --skip-keys="librealsense2 cv_bridge" \
    && colcon build --packages-up-to realsense2_camera --cmake-args -DBUILD_TESTING=OFF

# Python 3.11 and uv
RUN curl -fsSL -o /tmp/uv.tar.gz \
       https://github.com/astral-sh/uv/releases/download/0.12.8/uv-x86_64-unknown-linux-gnu.tar.gz \
    && echo "2e2b37e9811e17675a9e70bed5e1a58fc8c0388be63d751d72cc735188c149ff  /tmp/uv.tar.gz" \
       | sha256sum --check --strict \
    && tar -xzf /tmp/uv.tar.gz -C /tmp \
    && install /tmp/uv-x86_64-unknown-linux-gnu/uv /usr/local/bin/uv \
    && UV_PYTHON_INSTALL_DIR=/opt/python uv python install 3.11 \
    && UV_PYTHON_INSTALL_DIR=/opt/python uv venv --python 3.11 /opt/compute-venv
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# GraspGenX source and CPU patch
ARG GRASPGENX_REPOSITORY=https://github.com/NVlabs/GraspGenX.git
ARG GRASPGENX_REF=b9429097728cb1c430dd78b92edf17ba318aad03
COPY backend/patches/graspgenx-device.patch /tmp/graspgenx-device.patch
RUN git clone "$GRASPGENX_REPOSITORY" /opt/graspgenx \
    && git -C /opt/graspgenx checkout "$GRASPGENX_REF" \
    && git -C /opt/graspgenx apply /tmp/graspgenx-device.patch

# Shared CPU model dependencies, YOLOE assets and GraspGenX installation
ENV UV_TORCH_BACKEND=cpu \
    YOLO_CONFIG_DIR=/tmp \
    PERCEPTION_MODEL=/models/yoloe-26s-seg.pt \
    GRASPGENX_CHECKPOINT_DIR=/models/graspgenx \
    GRASPGENX_CHECKPOINT_ROOT=/models/graspgenx/release \
    GRASPGENX_GRIPPER_CFG_DIR=/grippers \
    GRASPGENX_GRIPPER_ASSETS=/grippers
COPY backend/services/perception-compute/pyproject.toml /tmp/perception-compute/pyproject.toml
RUN uv pip compile \
       /tmp/perception-compute/pyproject.toml /opt/graspgenx/pyproject.toml \
       --python /opt/compute-venv/bin/python \
       --no-emit-package perception-compute-service --no-emit-package graspgenx \
       --output-file /tmp/model-requirements.txt \
    && uv pip install --python /opt/compute-venv/bin/python \
       --requirements /tmp/model-requirements.txt \
    && uv pip install --python /opt/compute-venv/bin/python --no-deps /opt/graspgenx
COPY backend/services/perception-compute/models /models
COPY tools/graspgenx/build-description.py /opt/tools/graspgenx/build-description.py
RUN mkdir -p /service \
    && ln -s /models/mobileclip2_b.ts /service/mobileclip2_b.ts

# Rust toolchain
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
       | sh -s -- -y --default-toolchain 1.97.0
ENV PATH=/root/.cargo/bin:$PATH

# Dora runtime and replay tool
RUN git clone --filter=blob:none https://github.com/dora-rs/dora.git /dora-src \
    && git -C /dora-src checkout 06254f3d3353132f1184020d693680be4733bc2c \
    && cargo build --manifest-path /dora-src/Cargo.toml --release \
       -p dora-cli -p dora-replay-node \
    && ln -s /dora-src/target/release/dora /usr/local/bin/dora \
    && ln -s /dora-src/target/release/dora-replay-node /usr/local/bin/dora-replay-node

# StarArm-102 device support
RUN apt-get update \
    && apt-get install --yes \
       ros-lyrical-controller-manager \
       ros-lyrical-joint-state-broadcaster \
       ros-lyrical-joint-trajectory-controller
ARG STARARM_REPOSITORY=https://github.com/servodevelop/Star-Arm-102.git
ARG STARARM_REF=5979b346eb3a417840b29b76740754e4005d071a
COPY backend/devices/stararm-102 /opt/devices/stararm-102/project
RUN /opt/devices/stararm-102/project/docker/install-base.sh \
      "$STARARM_REPOSITORY" "$STARARM_REF"
