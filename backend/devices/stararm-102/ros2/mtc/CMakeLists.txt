cmake_minimum_required(VERSION 3.22)
project(stararm_102_mtc)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(fcl REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(kdl_parser REQUIRED)

add_library(stararm_mesh_self_filter SHARED src/mesh_self_filter.cpp)
target_compile_features(stararm_mesh_self_filter PUBLIC cxx_std_17)
target_link_libraries(stararm_mesh_self_filter PUBLIC
  moveit_ros_perception::moveit_pointcloud_octomap_updater_core
  pluginlib::pluginlib ${geometric_shapes_TARGETS} fcl)
pluginlib_export_plugin_description_file(moveit_ros_occupancy_map_monitor octomap_plugins.xml)
install(TARGETS stararm_mesh_self_filter LIBRARY DESTINATION lib)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickPlace.action"
  DEPENDENCIES geometry_msgs sensor_msgs
)

add_executable(pick_place_server src/pick_place_server.cpp)
target_compile_features(pick_place_server PUBLIC cxx_std_17)
target_link_libraries(pick_place_server PUBLIC
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${moveit_ros_planning_interface_TARGETS}
  ${moveit_task_constructor_core_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${shape_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_ros::tf2_ros
  ${geometric_shapes_TARGETS}
  fcl
  ${trac_ik_lib_TARGETS}
  ${kdl_parser_TARGETS}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(pick_place_server PRIVATE "${cpp_typesupport_target}")

install(TARGETS pick_place_server DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
