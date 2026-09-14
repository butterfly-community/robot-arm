"""Official scene-demo OBB offsets, in centimetres along the outward normal.

Negative enters the object; positive moves away. A revolute finger's extension
is geometry, not a reason to add outward stand-off. Keep the real URDF, open
state, sampling anchor and base-to-TCP transform unchanged.
Source: NVlabs/GraspGenX scripts/demo_scene_pc.py --moe_z_offsets_cm.
"""


def obb_z_offsets_cm(asset) -> tuple[float, ...]:
    """Use the official scene defaults for every gripper, without asset changes."""
    return (-2.0, 0.0)
