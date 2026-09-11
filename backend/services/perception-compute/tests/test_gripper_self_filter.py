import json
from types import SimpleNamespace

import numpy as np
import pytest

from perception_compute.gripper_self_filter import GripperSelfFilter


def test_measured_pose_and_joint_filter_only_environment(tmp_path):
    (tmp_path / "config.json").write_text(json.dumps({"tool_tcp_transform": np.eye(4).tolist()}))
    (tmp_path / "self-filter.json").write_text(json.dumps({"mesh_padding_offset_m": .005}))
    (tmp_path / "gripper.urdf").write_text('''<robot name="fixture">
      <link name="base"/><link name="finger"><collision><geometry>
      <sphere radius="0.01"/></geometry></collision></link>
      <joint name="finger_drive" type="prismatic"><parent link="base"/><child link="finger"/>
      <axis xyz="1 0 0"/><limit lower="0" upper="1" velocity="1" effort="1"/></joint>
      </robot>''')
    mask = GripperSelfFilter(tmp_path)
    observed = SimpleNamespace(tcp_pose=SimpleNamespace(position_m=[1., 2., 3.],
                       orientation_xyzw=[0., 0., 0., 1.]),
                       joint_positions_rad={"finger_drive": .03}, feedback_time_ns=123)
    points = np.array([[1.042, 2., 3.], [1.15, 2., 3.], [1., 2., 2.8]], dtype=np.float32)
    original = points.copy()
    np.testing.assert_allclose(mask.filter(points, observed), points[1:])
    np.testing.assert_array_equal(points, original)
    observed.joint_positions_rad["finger_drive"] = 0.
    np.testing.assert_allclose(mask.filter(points, observed), points)
    observed.joint_positions_rad = {}
    with pytest.raises(ValueError, match="URDF"):
        mask.filter(points, observed)
