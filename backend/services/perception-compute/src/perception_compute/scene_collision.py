"""Official sampled-surface distance predicate, evaluated with SciPy's exact KD-tree."""

import logging
import time

import numpy as np
from scipy.spatial import cKDTree

logger = logging.getLogger(__name__)


def filter_colliding_grasps(
    *, scene_pc, grasp_poses, gripper_surface_points, collision_threshold
):
    started = time.perf_counter()
    poses = np.asarray(grasp_poses, dtype=np.float32)
    if not len(poses):
        return np.zeros(0, dtype=bool)
    if not len(scene_pc):
        return np.ones(len(poses), dtype=bool)
    scene = np.asarray(scene_pc, dtype=np.float32)
    surface = np.asarray(gripper_surface_points, dtype=np.float32)
    world = np.einsum("kij,mj->kmi", poses[:, :3, :3], surface) + poses[:, None, :3, 3]
    # Exact Euclidean nearest neighbours (eps=0), same samples and strict < threshold.
    # Avoid the official dense (grasps * samples * scene_points) cdist intermediate.
    distances, _ = cKDTree(scene).query(
        world.reshape(-1, 3), k=1, eps=0,
        distance_upper_bound=collision_threshold, workers=-1,
    )
    keep = ~np.any(distances.reshape(len(poses), len(surface)) < collision_threshold, axis=1)
    logger.info(
        "[collision] %s/%s collision-free, exact KD-tree %.3f s",
        int(keep.sum()), len(keep), time.perf_counter() - started,
    )
    return keep
