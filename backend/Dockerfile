# syntax=docker/dockerfile:1.7
FROM base

# The backend project image builds the workspace once on top of backend-base.
WORKDIR /src
COPY . .
RUN source /opt/ros/lyrical/setup.bash \
    && source /opt/moveit_ws/install/setup.bash \
    && source /opt/devices/stararm-102/ros_ws/install/setup.bash \
    && cargo build --release --locked --workspace --no-default-features \
       --features perception-node/ros-runtime,perception-node/opencv-runtime,stararm-102-motion-node/ros-runtime \
    && target/release/perception-calibration --self-test
