FROM ubuntu:26.04

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
RUN mkdir -p /opt/build/downloads

# Shared by more than two backend services: source retrieval and native Rust builds.
RUN apt-get update \
    && apt-get install --yes ca-certificates curl git

RUN apt-get update \
    && apt-get install --yes \
       build-essential cmake libudev-dev pkg-config

# Rust toolchain
RUN curl --proto '=https' --tlsv1.2 -sSf -o /opt/build/downloads/rustup-init.sh \
       https://sh.rustup.rs \
    && sh /opt/build/downloads/rustup-init.sh -y --default-toolchain 1.97.0
ENV PATH=/root/.cargo/bin:$PATH

# Dora runtime and replay tool
RUN git clone https://github.com/dora-rs/dora.git /dora-src \
    && git -C /dora-src checkout 06254f3d3353132f1184020d693680be4733bc2c \
    && CARGO_HTTP_TIMEOUT=600 \
       CARGO_HTTP_LOW_SPEED_LIMIT=1 \
       cargo build --manifest-path /dora-src/Cargo.toml --release \
       -p dora-cli -p dora-replay-node \
    && ln -s /dora-src/target/release/dora /usr/local/bin/dora \
    && ln -s /dora-src/target/release/dora-replay-node /usr/local/bin/dora-replay-node
