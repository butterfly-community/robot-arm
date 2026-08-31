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

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickPlace.action"
  DEPENDENCIES geometry_msgs
)

add_executable(pick_place_server src/pick_place_server.cpp)
target_compile_features(pick_place_server PUBLIC cxx_std_17)
ament_target_dependencies(pick_place_server
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_task_constructor_core
  rclcpp
  rclcpp_action
  shape_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(pick_place_server "${cpp_typesupport_target}")

install(TARGETS pick_place_server DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
