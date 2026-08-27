#!/usr/bin/env python3
"""Create and finalize the JSON sidecar required for a Dora recording."""

from __future__ import annotations

import argparse
import json
import time
import urllib.request
from pathlib import Path
from typing import Any


DEFAULT_GATEWAY = "http://192.168.100.10:8765"
NAMESPACES = ("tracking", "spatial", "motion", "arm-execution")


def snapshot(base_url: str, namespace: str) -> dict[str, Any]:
    with urllib.request.urlopen(f"{base_url}/api/{namespace}/state") as response:
        return json.load(response)["values"]


def start(path: Path, base_url: str, sources: list[str]) -> None:
    states = {name: snapshot(base_url, name) for name in NAMESPACES}
    builds: dict[str, str] = {}
    configs: dict[str, int] = {}
    for namespace, values in states.items():
        for key, value in values.items():
            if not isinstance(value, dict):
                continue
            service = value if key.endswith("service_state") else value.get("service")
            if isinstance(service, dict):
                service_key = f"{namespace}/{key}"
                builds[service_key] = str(service.get("build_version", ""))
                configs[service_key] = int(service.get("config_version", 0))
    model = states["motion"].get("robot_model_info", {})
    spatial = states["spatial"].get("spatial_config_state", {})
    if isinstance(spatial, dict):
        configs["spatial-transform-node"] = int(spatial.get("config_version", 0))
    manifest = {
        "schema_version": 1,
        "build_versions": builds,
        "message_schema_version": 1,
        "model_id": model.get("model_id") if isinstance(model, dict) else None,
        "model_hash": (
            model.get("visualization", {}).get("manifest_hash")
            if isinstance(model, dict)
            else None
        ),
        "config_versions": configs,
        "coordinate_convention": "+X forward, +Y left, +Z up; quaternion XYZW; radians",
        "units": {"translation": "m", "rotation": "rad", "time": "ns"},
        "started_at_ns": time.time_ns(),
        "ended_at_ns": 0,
        "sources": sources,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")


def finish(path: Path) -> None:
    manifest = json.loads(path.read_text())
    manifest["ended_at_ns"] = time.time_ns()
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    start_parser = subparsers.add_parser("start")
    start_parser.add_argument("path", type=Path)
    start_parser.add_argument("--gateway", default=DEFAULT_GATEWAY)
    start_parser.add_argument("--source", action="append", default=[])
    finish_parser = subparsers.add_parser("finish")
    finish_parser.add_argument("path", type=Path)
    args = parser.parse_args()
    if args.command == "start":
        start(args.path, args.gateway.rstrip("/"), args.source)
    else:
        finish(args.path)


if __name__ == "__main__":
    main()
