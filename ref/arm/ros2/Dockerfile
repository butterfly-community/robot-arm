# syntax=docker/dockerfile:1.7
FROM moveit/moveit2:jazzy-release

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        ros-jazzy-joint-state-topic-hardware-interface \
        ros-jazzy-trac-ik-kinematics-plugin \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp/vendor
COPY --from=stararm_vendor ROS2_HUMBLE/src/stararm102_description \
    ROS2_HUMBLE/src/stararm102_description
COPY --from=stararm_vendor ROS2_HUMBLE/src/stararm102_moveit_config \
    ROS2_HUMBLE/src/stararm102_moveit_config
COPY arm/patches/star-arm-102-fl-moveit-model.patch \
    arm/patches/star-arm-102-fl-moveit-dynamics.patch \
    arm/patches/star-arm-102-fl-topic-io.patch \
    /tmp/patches/

RUN git apply --ignore-space-change --ignore-whitespace \
        /tmp/patches/star-arm-102-fl-moveit-model.patch \
    && git apply --ignore-space-change --ignore-whitespace \
        /tmp/patches/star-arm-102-fl-moveit-dynamics.patch \
    && git apply --ignore-space-change --ignore-whitespace \
        /tmp/patches/star-arm-102-fl-topic-io.patch \
    && mkdir -p /opt/stararm_ws/src \
    && mv ROS2_HUMBLE/src/stararm102_description /opt/stararm_ws/src/ \
    && mv ROS2_HUMBLE/src/stararm102_moveit_config /opt/stararm_ws/src/

COPY arm/ros2/stararm102_teleop_moveit \
    /opt/stararm_ws/src/stararm102_teleop_moveit

WORKDIR /opt/stararm_ws
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build \
        --packages-select \
            stararm102_description \
            stararm102_moveit_config \
            stararm102_teleop_moveit \
        --event-handlers console_direct+ \
    && rm -rf /tmp/patches /tmp/vendor log

COPY --chmod=755 arm/ros2/start-moveit.sh /usr/local/bin/start-moveit
ENTRYPOINT ["/usr/local/bin/start-moveit"]
