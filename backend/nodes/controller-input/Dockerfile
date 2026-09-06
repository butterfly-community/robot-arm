FROM robot-arm-services-backend-base:2026.09.05-r9 AS build

RUN apt-get update \
    && apt-get install --yes libusb-1.0-0-dev
WORKDIR /src
COPY backend .
RUN cargo build --release --locked -p controller-input-node

FROM robot-arm-services-backend-runtime:2026.09.05-r3
RUN apt-get update \
    && apt-get install --yes libusb-1.0-0
COPY --from=build \
     /src/target/release/controller-input-node \
     /usr/local/bin/controller-input-node
