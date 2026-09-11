"""Derive OBB sampling coverage from a gripper's actual opening kinematics.

The official sampler anchors OPEN fingertips on each OBB face. Revolute fingers
can extend farther as they close. Cover that stand-off too, without altering the
asset, TCP, returned transforms, orientations, or scene collision filtering.
"""
import json
from pathlib import Path

import numpy as np
import trimesh
import yourdfpy


def obb_z_offsets_cm(asset: Path) -> tuple[float, ...]:
    asset = Path(asset)
    config = json.loads((asset / "config.json").read_text())
    robot = yourdfpy.URDF.load(str(asset / "gripper.urdf"))
    states = {}
    for key in ("open", "close"):
        robot.update_cfg(config[key])
        states[key] = {node: robot.scene.graph.get(node)[0].copy()
                       for node in robot.scene.graph.nodes_geometry}
    moving = [node for node in states["open"]
              if not np.allclose(states["open"][node], states["close"][node])]
    fronts = {}
    for key in states:
        fronts[key] = max(
            float(trimesh.transform_points(
                robot.scene.geometry[robot.scene.graph.get(node)[1]].vertices,
                states[key][node])[:, 2].max()) for node in moving
        )
    extension_cm = max(0.0, fronts["close"] - fronts["open"]) * 100
    # Keep both official scene offsets. The additional endpoint derives solely
    # from the URDF; a parallel gripper with no axial extension adds no offset.
    return tuple(dict.fromkeys((-2.0, 0.0, extension_cm)))
