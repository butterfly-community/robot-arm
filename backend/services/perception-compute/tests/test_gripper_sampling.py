import json

import numpy as np
import pytest

from perception_compute.gripper_sampling import obb_z_offsets_cm


@pytest.mark.parametrize("kind,axis,opened,expected", [
    ("prismatic", "1 0 0", .05, (-2.0, 0.0)),
    ("revolute", "0 1 0", np.pi / 3, (-2.0, 0.0, 10-(10*.5+.5*np.sqrt(3)/2))),
])
def test_offsets_follow_moving_mesh_not_fixed_palm(tmp_path, kind, axis, opened, expected):
    (tmp_path / "config.json").write_text(json.dumps({
        "open": {"drive": opened}, "close": {"drive": 0.0}}))
    (tmp_path / "gripper.urdf").write_text(f'''<robot name="fixture">
      <link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
      <link name="finger"><visual><origin xyz="0 0 0.05"/><geometry>
        <box size="0.01 0.01 0.1"/></geometry></visual></link>
      <joint name="drive" type="{kind}"><parent link="base"/><child link="finger"/>
        <axis xyz="{axis}"/><limit lower="0" upper="2" velocity="1" effort="1"/>
      </joint></robot>''')
    assert obb_z_offsets_cm(tmp_path) == pytest.approx(expected)
