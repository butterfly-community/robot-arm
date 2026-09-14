"""Official scene offsets: no geometry-derived outward stand-off."""
import numpy as np
import pytest

from perception_compute.gripper_sampling import obb_z_offsets_cm


def test_offsets_are_official_scene_defaults_for_every_asset(tmp_path):
    assert obb_z_offsets_cm(tmp_path) == (-2.0, 0.0)
    assert obb_z_offsets_cm(tmp_path / "another-gripper") == (-2.0, 0.0)
    assert all(offset <= 0 for offset in obb_z_offsets_cm(tmp_path))


@pytest.mark.parametrize("outward", [(0., 0., 1.), (1., 0., 0.), (0., -1., 0.)])
def test_official_sampler_offset_sign_and_real_tcp_are_separate(tmp_path, outward):
    from graspgenx.samplers.graspmoe import _build_face_candidates

    n = np.array(outward)
    in_plane = np.array([0., 1., 0.]) if n[1] == 0 else np.array([1., 0., 0.])
    face = np.array([.2, .1, .04])
    sampling_depth = .066470974
    real_tcp_depth = .09338
    poses = _build_face_candidates(
        face_origin_world=face, approach_dir_world=n,
        in_plane_axis_world=in_plane, positions_local=np.array([0.]),
        yaws=np.array([0.]), z_offsets_m=np.array(obb_z_offsets_cm(tmp_path)) / 100,
        gripper_depth_m=sampling_depth,
    )
    tool = np.eye(4)
    tool[2, 3] = real_tcp_depth
    tcp = poses @ tool
    # -20 mm moves INTO each face, not always down the world's Z axis.
    np.testing.assert_allclose(tcp[0, :3, 3] - tcp[1, :3, 3], -.02*n, atol=1e-7)
    # Sampling never changes the real base-to-TCP calibration.
    np.testing.assert_allclose(tcp[1, :3, 3], face-(real_tcp_depth-sampling_depth)*n, atol=1e-7)
