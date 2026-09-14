import numpy as np
from scipy.spatial.transform import Rotation

from perception_compute.grasp_selection import MAX_GRASP_CANDIDATES, representative_grasps


def corners():
    return np.array([[x, y, z] for x in (-0.04, 0.04)
                     for y in (-0.02, 0.02) for z in (0.0, 0.12)])


def proposals():
    count = MAX_GRASP_CANDIDATES
    poses = np.tile(np.eye(4), (count * 10, 1, 1))
    # Well-separated orientations, 10 very close scored copies each.
    angles = np.repeat(np.arange(count) * (360.0 / count), 10) + np.tile(np.arange(10), count) * 0.001
    poses[:, :3, :3] = Rotation.from_euler("z", angles[:, None], degrees=True).as_matrix()
    return poses, np.tile(np.linspace(0.1, 0.9, 10), count)


def test_nearby_orientations_keep_the_best_original_in_each_group():
    poses, scores = proposals()
    original = poses.copy()
    selected = representative_grasps(poses, scores, corners())
    np.testing.assert_array_equal(selected, np.arange(9, MAX_GRASP_CANDIDATES * 10, 10))
    np.testing.assert_array_equal(poses, original)


def test_same_result_in_a_different_rigid_scene_frame():
    poses, scores = proposals()
    frame = np.eye(4)
    frame[:3, :3] = Rotation.from_euler("xyz", [0.3, -0.7, 1.2]).as_matrix()
    frame[:3, 3] = [0.7, -0.5, 1.2]
    np.testing.assert_array_equal(
        representative_grasps(frame @ poses, scores, corners()),
        representative_grasps(poses, scores, corners()))


def test_same_angle_at_distinct_contact_positions_remains_distinct():
    count = MAX_GRASP_CANDIDATES
    poses = np.tile(np.eye(4), (count * 3, 1, 1))
    poses[:, 0, 3] = np.repeat(np.arange(count) * 0.01, 3)
    scores = np.tile([0.1, 0.9, 0.5], count)
    np.testing.assert_array_equal(
        representative_grasps(poses, scores, corners()), np.arange(1, count * 3, 3))


def test_under_budget_is_unchanged_including_empty():
    poses, scores = proposals()
    for count in (0, 1, MAX_GRASP_CANDIDATES):
        np.testing.assert_array_equal(
            representative_grasps(poses[:count], scores[:count], corners()), np.arange(count))
