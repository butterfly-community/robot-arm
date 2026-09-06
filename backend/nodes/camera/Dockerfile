FROM robot-arm-services-backend-base:2026.09.05-r9 AS build

# OpenCV is owned by camera-node for calibration.
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

# RealSense is owned only by camera-node.
RUN apt-get update \
    && apt-get install --yes gnupg libusb-1.0-0-dev \
    && install -d /etc/apt/keyrings \
    && curl --fail --silent --show-error --location \
       --retry 5 --retry-all-errors --retry-delay 2 \
       https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
       | gpg --dearmor --output /etc/apt/keyrings/librealsenseai.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo resolute main" \
       > /etc/apt/sources.list.d/librealsense.list \
    && apt-get update \
    && apt-get install --yes librealsense2-dev librealsense2-utils

WORKDIR /src
COPY backend .
RUN cargo build --release --locked -p camera-node \
       --features realsense-runtime,opencv-runtime

FROM robot-arm-services-backend-runtime:2026.09.05-r3
COPY --from=build /etc/apt/keyrings/librealsenseai.gpg \
     /etc/apt/keyrings/librealsenseai.gpg
COPY --from=build /etc/apt/sources.list.d/librealsense.list \
     /etc/apt/sources.list.d/librealsense.list
RUN apt-get update \
    && apt-get install --yes --no-install-recommends librealsense2
COPY --from=build /opt/opencv5 /opt/opencv5
ENV LD_LIBRARY_PATH=/opt/opencv5/lib
COPY --from=build /src/target/release/camera-node \
     /usr/local/bin/camera-node
