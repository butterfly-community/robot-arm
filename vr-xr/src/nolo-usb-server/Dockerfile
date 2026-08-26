# syntax=docker/dockerfile:1
FROM denoland/deno:debian-2.9.5 AS viewer

USER root
RUN apt-get update \
    && apt-get install --yes --no-install-recommends git \
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
        /tmp/patches/star-arm-102-fl-topic-io.patch

WORKDIR /tmp/viewer
COPY vr-xr/src/controller-viewer/public public
COPY vr-xr/tools/prepare-controller-viewer.ts /tmp/tools/
RUN deno run --allow-read --allow-write \
    /tmp/tools/prepare-controller-viewer.ts \
    /tmp/vendor/ROS2_HUMBLE/src/stararm102_description \
    /tmp/viewer/public

FROM rust:bookworm AS build

RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        libudev-dev \
        libusb-1.0-0-dev \
        pkg-config \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /source
COPY arm/config/stararm102-fl.v1.json arm/config/stararm102-fl.v1.json
COPY arm/src/fashionstar-uart arm/src/fashionstar-uart
COPY arm/src/stararm102-control arm/src/stararm102-control
COPY vr-xr/src/nolo-usb-server vr-xr/src/nolo-usb-server
RUN cargo build \
    --locked \
    --release \
    --manifest-path vr-xr/src/nolo-usb-server/Cargo.toml \
    --bin nolo-usb-server

FROM debian:bookworm-slim

RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        libudev1 \
        libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build \
    /source/vr-xr/src/nolo-usb-server/target/release/nolo-usb-server \
    /usr/local/bin/nolo-usb-server
COPY --from=viewer /tmp/viewer/public /srv/controller-viewer

ENTRYPOINT ["/usr/local/bin/nolo-usb-server"]
