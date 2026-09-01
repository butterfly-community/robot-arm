# syntax=docker/dockerfile:1.7
FROM ubuntu:26.04
ARG NODE_VERSION=24.20.0
ENV NEXT_TELEMETRY_DISABLED=1
RUN apt-get update \
    && apt-get install --yes ca-certificates curl xz-utils \
    && curl -fsSL "https://nodejs.org/dist/v${NODE_VERSION}/node-v${NODE_VERSION}-linux-x64.tar.xz" \
       | tar -xJ --strip-components=1 -C /usr/local
RUN corepack enable && corepack prepare pnpm@11.25.0 --activate
