# Star Arm 102 visualization assets

The URDF geometry and nine STL meshes in this directory are copied without
geometric modification from the local upstream checkout:

- repository: `servodevelop/Star-Arm-102`
- commit: `5979b346eb3a417840b29b76740754e4005d071a`
- source: `ROS2_HUMBLE/src/stararm102_description/{urdf,meshes}`
- upstream README license declaration: MIT

Only the visualization files needed by the read-only browser simulator are
retained. The simulator ignores collision, inertia, transmission and
hardware-control data.  The seven `<limit>` ranges in the retained URDF are
overridden with the confirmed Star Arm 102-FL product/model ranges: J1 ±110°,
J2 0–180°, J3 -270–0° in the URDF axis convention, J4 ±90°, J5 ±65°,
J6 ±150°, and the active `joint7_left` gripper 0–90°.  `joint7_right` remains
the `multiplier=-1` mimic joint.
