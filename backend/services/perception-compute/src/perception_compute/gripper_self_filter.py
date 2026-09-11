"""Remove the observed end effector from GraspGenX's environment, not its target.

Same FCL sphere/BVH surface-distance query and device padding as MoveIt's self
filter. URDF FK/mimic joints come from yourdfpy; no custom kinematics/geometry.
"""
import json
import logging
import time
from pathlib import Path

import fcl
import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy

logger = logging.getLogger(__name__)


class GripperSelfFilter:
    def __init__(self, asset):
        asset = Path(asset)
        config = json.loads((asset / "config.json").read_text())
        self.tcp_to_base = np.linalg.inv(np.asarray(config["tool_tcp_transform"]))
        self.padding = float(json.loads((asset / "self-filter.json").read_text())["mesh_padding_offset_m"])
        if not np.isfinite(self.padding) or self.padding <= 0:
            raise ValueError("Device mesh self-filter distance must be finite and positive")
        self.robot = yourdfpy.URDF.load(str(asset / "gripper.urdf"),
            build_collision_scene_graph=True, load_collision_meshes=True)
        self.models = {}
        for name, mesh in self.robot.collision_scene.geometry.items():
            geometry = fcl.BVHModel()
            geometry.beginModel(len(mesh.vertices), len(mesh.faces))
            geometry.addSubModel(np.asarray(mesh.vertices), np.asarray(mesh.faces, dtype=np.int32))
            geometry.endModel()
            self.models[name] = (fcl.CollisionObject(geometry), mesh.bounds.copy())
        self.probe = fcl.CollisionObject(fcl.Sphere(self.padding))
        self.request = fcl.CollisionRequest()

    def filter(self, points, observation):
        started = time.perf_counter()
        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        if set(observation.joint_positions_rad) != set(self.robot.actuated_joint_names):
            raise ValueError("Observed gripper joints do not match its URDF actuated joints")
        if not np.isfinite(list(observation.joint_positions_rad.values())).all():
            raise ValueError("Observed gripper joints must be finite")
        # yourdfpy's revolute/mimic implementation calls q.item().
        self.robot.update_cfg({key: np.float64(value) for key, value in observation.joint_positions_rad.items()})
        pose = observation.tcp_pose
        world_from_tcp = np.eye(4)
        world_from_tcp[:3, :3] = Rotation.from_quat(pose.orientation_xyzw).as_matrix()
        world_from_tcp[:3, 3] = pose.position_m
        world_from_base = world_from_tcp @ self.tcp_to_base
        keep = np.ones(len(points), dtype=bool)
        scene = self.robot.collision_scene
        for node in scene.graph.nodes_geometry:
            transform, name = scene.graph.get(node)
            world_from_mesh = world_from_base @ transform
            model, bounds = self.models[name]
            local = (points-world_from_mesh[:3, 3]) @ world_from_mesh[:3, :3]
            indices = np.flatnonzero(keep & (local >= bounds[0]-self.padding).all(axis=1)
                                          & (local <= bounds[1]+self.padding).all(axis=1))
            for index in indices:
                self.probe.setTranslation(local[index])
                if fcl.collide(model, self.probe, self.request, fcl.CollisionResult()):
                    keep[index] = False
        logger.info("[self-filter] observed gripper removed %s/%s environment points in %.3f s, feedback=%s",
                    int((~keep).sum()), len(points), time.perf_counter()-started, observation.feedback_time_ns)
        return points[keep]
