FROM robot-arm-services-backend-base:2026.09.05-r9 AS build

WORKDIR /src
COPY backend .
RUN cargo build --release --locked -p web-gateway-node

FROM robot-arm-services-backend-runtime:2026.09.05-r3
COPY --from=build \
     /src/target/release/web-gateway-node \
     /usr/local/bin/web-gateway-node
