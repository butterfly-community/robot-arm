FROM robot-arm-services-backend-base:2026.09.05-r9 AS build

WORKDIR /src
COPY backend .
RUN cargo build --release --locked -p stararm-102-execution-node

FROM robot-arm-services-stararm-102-base:2026.09.05-r2 AS device

FROM robot-arm-services-backend-runtime:2026.09.05-r3
# Execution serves the same patched URDF/meshes to the browser; no ROS runtime.
COPY --from=device /opt/devices/stararm-102/vendor/ROS2_HUMBLE/src/stararm102_description \
     /opt/devices/stararm-102/vendor/ROS2_HUMBLE/src/stararm102_description
COPY --from=build /src/target/release/stararm-102-execution-node \
     /usr/local/bin/stararm-102-execution-node
