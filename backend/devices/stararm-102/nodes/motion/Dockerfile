FROM robot-arm-services-stararm-102-base:2026.09.13-r1 AS device

FROM robot-arm-services-backend-base:2026.09.05-r9 AS build

RUN mkdir -p /opt/build/downloads /opt/build/patches

# ROS 2 and MoveIt are owned only by the motion service.
RUN curl -fsSL -o /opt/build/downloads/ros2-apt-source.deb \
       https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.resolute_all.deb \
    && echo "a275b9b819874e745a928e83e39c429fa4d607159285c4ef3ebcf75afa732ee3  /opt/build/downloads/ros2-apt-source.deb" \
       | sha256sum --check --strict \
    && apt-get install --yes /opt/build/downloads/ros2-apt-source.deb
RUN apt-get update \
    && apt-get install --yes \
       libclang-dev \
       ros-dev-tools \
       ros-lyrical-controller-manager \
       ros-lyrical-joint-state-broadcaster \
       ros-lyrical-joint-state-topic-hardware-interface \
       ros-lyrical-joint-trajectory-controller \
       ros-lyrical-moveit-configs-utils \
       ros-lyrical-moveit-planners-ompl \
       ros-lyrical-moveit-ros-move-group \
       ros-lyrical-moveit-ros-perception \
       ros-lyrical-moveit-ros-visualization \
       ros-lyrical-moveit-servo \
       ros-lyrical-moveit-simple-controller-manager \
       ros-lyrical-moveit-task-constructor-capabilities \
       ros-lyrical-moveit-task-constructor-core \
       ros-lyrical-moveit-task-constructor-msgs \
       ros-lyrical-moveit-task-constructor-visualization \
       ros-lyrical-ros-base \
       ros-lyrical-trac-ik-kinematics-plugin \
    && rosdep init \
    && rosdep update --rosdistro lyrical
ENV ROS_DISTRO=lyrical

# Project patch for the one upstream MoveIt Servo defect used by this arm.
ARG MOVEIT_REPOSITORY=https://github.com/moveit/moveit2.git
ARG MOVEIT_REF=2.15.0
COPY backend/patches/moveit-servo-halt-pose-at-joint-bound.patch \
     /opt/build/patches/moveit-servo.patch
RUN git clone --depth=1 --branch "$MOVEIT_REF" "$MOVEIT_REPOSITORY" \
       /opt/moveit_ws/src/moveit2 \
    && git -C /opt/moveit_ws/src/moveit2 apply \
       --ignore-space-change --ignore-whitespace /opt/build/patches/moveit-servo.patch \
    && source /opt/ros/lyrical/setup.bash \
    && cd /opt/moveit_ws \
    && colcon build --packages-select moveit_servo \
       --cmake-args -DBUILD_TESTING=OFF

# Build the patched StarArm ROS model and the device-independent Rust adapter.
COPY --from=device /opt/devices/stararm-102/vendor /opt/devices/stararm-102/vendor
COPY backend/devices/stararm-102/ros2 /opt/devices/stararm-102/project/ros2
COPY backend/devices/stararm-102/docker/install-ros.sh \
     /opt/devices/stararm-102/project/docker/install-ros.sh
RUN /opt/devices/stararm-102/project/docker/install-ros.sh
WORKDIR /src
COPY backend .
RUN source /opt/ros/lyrical/setup.bash \
    && source /opt/moveit_ws/install/setup.bash \
    && source /opt/devices/stararm-102/ros_ws/install/setup.bash \
    && cargo build --release --locked -p stararm-102-motion-node \
       --no-default-features --features ros-runtime

FROM robot-arm-services-backend-runtime:2026.09.05-r3

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
RUN mkdir -p /opt/build/downloads
COPY --from=build /opt/build/downloads/ros2-apt-source.deb \
     /opt/build/downloads/ros2-apt-source.deb
RUN apt-get install --yes /opt/build/downloads/ros2-apt-source.deb \
    && apt-get update \
    && apt-get install --yes \
       fonts-noto-cjk novnc openbox tigervnc-standalone-server websockify \
       ros-lyrical-controller-manager \
       ros-lyrical-joint-state-broadcaster \
       ros-lyrical-joint-state-topic-hardware-interface \
       ros-lyrical-joint-trajectory-controller \
       ros-lyrical-moveit-configs-utils \
       ros-lyrical-moveit-planners-ompl \
       ros-lyrical-moveit-ros-move-group \
       ros-lyrical-moveit-ros-perception \
       ros-lyrical-moveit-ros-visualization \
       ros-lyrical-moveit-servo \
       ros-lyrical-moveit-simple-controller-manager \
       ros-lyrical-moveit-task-constructor-capabilities \
       ros-lyrical-moveit-task-constructor-core \
       ros-lyrical-moveit-task-constructor-msgs \
       ros-lyrical-moveit-task-constructor-visualization \
       ros-lyrical-ros-base \
       ros-lyrical-trac-ik-kinematics-plugin
ENV ROS_DISTRO=lyrical
COPY --from=build /opt/moveit_ws/install /opt/moveit_ws/install
COPY --from=build /opt/devices/stararm-102 /opt/devices/stararm-102
COPY backend/devices/stararm-102/docker/rviz-index.html \
     /usr/share/novnc/index.html
COPY --from=build /src/target/release/stararm-102-motion-node \
     /usr/local/bin/stararm-102-motion-node
COPY backend/devices/stararm-102/docker/motion-entrypoint.sh \
     /usr/local/bin/motion-entrypoint.sh
