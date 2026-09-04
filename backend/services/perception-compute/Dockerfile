FROM robot-arm-services-backend-base:2026.09.04-r1
WORKDIR /service
COPY . .
RUN uv pip install --python /opt/compute-venv/bin/python --no-deps .
EXPOSE 8000
CMD ["/opt/compute-venv/bin/uvicorn", "perception_compute.app:app", "--host", "0.0.0.0", "--port", "8000"]
