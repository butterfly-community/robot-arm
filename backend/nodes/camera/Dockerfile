FROM robot-arm-services-backend-base:2026.09.05-r9 AS build-deps

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
       -DBUILD_LIST=core,calib,geometry,imgcodecs,imgproc,objdetect,stereo \
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

# Runtime must install the exact ABI that the Rust binary links against. An
# independently cached apt layer can otherwise retain an older SONAME.
RUN dpkg-query -W -f='${Version}' librealsense2 > /opt/realsense-version

RUN apt-get update && apt-get install --yes --no-install-recommends \
    libavcodec-dev libavformat-dev libavutil-dev libswscale-dev

FROM build-deps AS build
WORKDIR /src
COPY backend .
RUN cargo build --release --locked -p camera-node \
       --features realsense-runtime,opencv-runtime

FROM robot-arm-services-backend-runtime:2026.09.05-r3
COPY --from=build /etc/apt/keyrings/librealsenseai.gpg \
     /etc/apt/keyrings/librealsenseai.gpg
COPY --from=build /etc/apt/sources.list.d/librealsense.list \
     /etc/apt/sources.list.d/librealsense.list
COPY --from=build /opt/realsense-version /opt/robot-arm/realsense-version
RUN apt-get update \
    && apt-get install --yes --no-install-recommends "librealsense2=$(cat /opt/robot-arm/realsense-version)" \
       libavcodec62 libavformat62 libavutil60 libswscale9
COPY --from=build /opt/opencv5 /opt/opencv5
ENV LD_LIBRARY_PATH=/opt/opencv5/lib
COPY backend/nodes/camera/assets/stereo /opt/robot-arm/stereo
ENV CAMERA_STEREO_ASSETS=/opt/robot-arm/stereo
COPY --from=build /src/target/release/camera-node \
     /usr/local/bin/camera-node
RUN ldd /usr/local/bin/camera-node \
    | awk '/not found/ { missing=1; print } END { exit missing }'
