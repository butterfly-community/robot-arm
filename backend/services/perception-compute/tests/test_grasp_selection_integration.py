"""Model boundary preserves filtered proposal provenance and canonical→TCP order."""
import sys
import threading
from types import SimpleNamespace

import numpy as np
import trimesh

from perception_compute.app import GraspGenXBackend, GraspRequest
from perception_compute.grasp_selection import MAX_GRASP_CANDIDATES, representative_grasps


def test_model_returns_representatives_of_environment_filtered_poses(monkeypatch):
    count = MAX_GRASP_CANDIDATES * 3
    poses = np.tile(np.eye(4), (count, 1, 1))
    poses[:, 0, 3] = np.arange(count) * 0.001
    scores = np.linspace(0.1, 0.9, count)
    branches = [f"source-{i}" for i in range(count)]
    tool = np.eye(4)
    tool[2, 3] = 0.09338
    mesh = trimesh.creation.box(extents=[0.14, 0.06, 0.07])
    sampler = SimpleNamespace(gripper=SimpleNamespace(tool_tcp_transform=tool, collision_mesh=mesh))
    backend = GraspGenXBackend.__new__(GraspGenXBackend)
    backend._lock = threading.Lock()
    backend._samplers = {"fixture": (sampler, np.zeros((2000, 3)), (0.0,))}

    def planner(*args, **kwargs):
        return [(poses, scores, branches, None)]

    keep = np.arange(count) % 3 != 0

    def collision_filter(**kwargs):
        np.testing.assert_array_equal(kwargs["grasp_poses"], poses)
        return keep

    monkeypatch.setitem(sys.modules, "graspgenx.samplers",
                        SimpleNamespace(run_planner_on_batch=planner))
    monkeypatch.setattr("perception_compute.scene_collision.filter_colliding_grasps", collision_filter)
    result = backend.infer(GraspRequest(
        points_xyz_m=[(0.1, 0.2, 0.3)], scene_points_xyz_m=[(0.0, 0.0, 0.0)],
        gripper_asset_id="fixture", observed_gripper=None, collision_threshold_m=0.01,
    ))
    selected = representative_grasps(poses[keep], scores[keep], trimesh.bounds.corners(mesh.bounds))
    original_indices = np.flatnonzero(keep)[selected]
    assert len(result.candidates) == MAX_GRASP_CANDIDATES == 100
    for candidate, index in zip(result.candidates, original_indices, strict=True):
        np.testing.assert_allclose(candidate.transform, poses[index] @ tool)
        assert candidate.confidence == scores[index]
        assert candidate.branch == branches[index]
