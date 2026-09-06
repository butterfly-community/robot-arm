FROM robot-arm-services-backend-base:2026.09.05-r9 AS build-base

FROM ubuntu:26.04

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# Shared by more than two services: Dora and the Rust executable runtime.
RUN apt-get update \
    && apt-get install --yes \
       ca-certificates libgcc-s1 zlib1g

# Dora runtime executables.
COPY --from=build-base \
     /dora-src/target/release/dora \
     /dora-src/target/release/dora-replay-node \
     /usr/local/bin/
COPY backend/docker/dora-entrypoint.sh /usr/local/bin/dora-entrypoint.sh
