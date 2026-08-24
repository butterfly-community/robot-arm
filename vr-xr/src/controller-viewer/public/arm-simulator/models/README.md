# Star Arm 102 visualization assets

The URDF and nine STL meshes in this directory are copied without geometric
modification from the local upstream checkout:

- repository: `servodevelop/Star-Arm-102`
- commit: `5979b346e6339d96f91a1ade1ab71933cf7dad36`
- source: `ROS2_HUMBLE/src/stararm102_description/{urdf,meshes}`
- upstream README license declaration: MIT

Only the visualization files needed by the read-only browser simulator are
retained. The simulator ignores collision, inertia, transmission and
hardware-control data.
