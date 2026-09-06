FROM robot-arm-services-perception-compute-base:2026.09.06-r5 AS build
WORKDIR /service
COPY . .
RUN uv pip install --python /opt/compute-venv/bin/python \
       --target /runtime-root/service-packages --no-deps .

FROM robot-arm-services-perception-compute-runtime:2026.09.06-r5
WORKDIR /service
ENV PYTHONPATH=/service-packages
COPY --from=build /runtime-root /
EXPOSE 8000
CMD ["/opt/compute-venv/bin/uvicorn", "perception_compute.app:app", "--host", "0.0.0.0", "--port", "8000"]
