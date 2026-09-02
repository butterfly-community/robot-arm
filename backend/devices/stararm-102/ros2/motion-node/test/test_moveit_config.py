from stararm_102_motion_node.moveit_config import build_moveit_configs


def test_point_cloud_octomap_configuration_is_loaded():
    sensors = build_moveit_configs(0.005).sensors_3d

    assert sensors["octomap_frame"] == "base_link"
    assert sensors["octomap_resolution"] == 0.005
    assert sensors["sensors"] == ["perception_point_cloud"]
    assert sensors["perception_point_cloud"]["sensor_plugin"] == (
        "occupancy_map_monitor/PointCloudOctomapUpdater"
    )
    assert sensors["perception_point_cloud"]["point_cloud_topic"] == (
        "/perception/depth/points"
    )
