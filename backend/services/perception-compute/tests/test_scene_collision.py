import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from perception_compute.scene_collision import filter_colliding_grasps


@pytest.mark.parametrize("seed", range(5))
def test_exact_nearest_distance_matches_exhaustive_predicate(seed):
    rng = np.random.default_rng(seed)
    scene = rng.uniform(-0.1, 0.1, (256, 3)).astype(np.float32)
    surface = rng.uniform(-0.04, 0.04, (100, 3)).astype(np.float32)
    poses = np.repeat(np.eye(4, dtype=np.float32)[None], 20, axis=0)
    poses[:, :3, :3] = Rotation.random(20, random_state=rng).as_matrix()
    poses[:, :3, 3] = rng.uniform(-0.15, 0.15, (20, 3))
    world = np.einsum("kij,mj->kmi", poses[:, :3, :3], surface) + poses[:, None, :3, 3]
    distances = np.linalg.norm(world.astype(float)[:, :, None] - scene[None, None], axis=-1)
    expected = ~np.any(distances < 0.005, axis=(1, 2))
    actual = filter_colliding_grasps(scene_pc=scene, grasp_poses=poses,
                                   gripper_surface_points=surface, collision_threshold=0.005)
    np.testing.assert_array_equal(actual, expected)


def test_strict_threshold_and_empty_inputs():
    # Exactly representable coordinates: contact below, on and above threshold.
    poses = np.repeat(np.eye(4, dtype=np.float32)[None], 3, axis=0)
    poses[:, 0, 3] = [0.0625, 0.125, 0.25]
    options = dict(scene_pc=np.zeros((1, 3)), grasp_poses=poses,
                   gripper_surface_points=np.zeros((1, 3)), collision_threshold=0.125)
    np.testing.assert_array_equal(filter_colliding_grasps(**options), [False, True, True])
    options["scene_pc"] = np.empty((0, 3))
    np.testing.assert_array_equal(filter_colliding_grasps(**options), [True, True, True])
    options["grasp_poses"] = np.empty((0, 4, 4))
    assert filter_colliding_grasps(**options).shape == (0,)
