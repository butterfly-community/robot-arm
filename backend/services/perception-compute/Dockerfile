FROM robot-arm-services-perception-compute-base:2026.09.07-r7 AS build
ADD --checksum=sha256:a361bcbd0b110daf0132d7939262577486d021961cda34428055494fc221c4c7 \
    https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26x-seg-pf.pt \
    /runtime-root/models/yoloe-26x-seg-pf.pt
WORKDIR /service
COPY . .
RUN uv pip install --python /opt/compute-venv/bin/python \
       --target /runtime-root/service-packages --no-deps .

FROM robot-arm-services-perception-compute-runtime:2026.09.07-r7
WORKDIR /service
ENV PYTHONPATH=/service-packages
COPY --from=build /runtime-root /
EXPOSE 8000
CMD ["/opt/compute-venv/bin/uvicorn", "perception_compute.app:app", "--host", "0.0.0.0", "--port", "8000"]
