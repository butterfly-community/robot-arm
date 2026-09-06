FROM robot-arm-services-backend-base:2026.09.05-r9 AS build

# OpenCV is owned by scene-node for instance geometry.
RUN apt-get update \
    && apt-get install --yes \
       clang g++-16 libclang-dev ninja-build
ENV OpenCV_DIR=/opt/opencv5/lib/cmake/opencv5 \
    LD_LIBRARY_PATH=/opt/opencv5/lib \
    PKG_CONFIG_PATH=/opt/opencv5/lib/pkgconfig \
    OPENCV_DISABLE_PROBES=pkg_config
RUN git clone --depth=1 --branch 5.0.0 https://github.com/opencv/opencv.git /opencv \
    && cmake -S /opencv -B /opencv-build -G Ninja \
       -DCMAKE_BUILD_TYPE=Release \
       -DCMAKE_INSTALL_PREFIX=/opt/opencv5 \
       -DBUILD_LIST=core,calib,geometry,imgcodecs,imgproc,objdetect \
       -DBUILD_opencv_apps=OFF \
       -DBUILD_PERF_TESTS=OFF \
       -DBUILD_TESTS=OFF \
       -DBUILD_EXAMPLES=OFF \
    && cmake --build /opencv-build \
    && cmake --install /opencv-build

WORKDIR /src
COPY backend .
RUN cargo build --release --locked -p scene-node

FROM robot-arm-services-backend-runtime:2026.09.05-r3
RUN apt-get update \
    && apt-get install --yes \
       libstdc++6
COPY --from=build /opt/opencv5 /opt/opencv5
ENV LD_LIBRARY_PATH=/opt/opencv5/lib
COPY --from=build /src/target/release/scene-node \
     /usr/local/bin/scene-node
