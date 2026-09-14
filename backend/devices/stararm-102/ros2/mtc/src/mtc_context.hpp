#pragma once
#include <Eigen/Geometry>
#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>
#include "stararm_102_mtc/action/pick_place.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace stararm_mtc {
namespace mtc = moveit::task_constructor;
using PickPlace = stararm_102_mtc::action::PickPlace;
constexpr char kArmGroup[] = "arm";
constexpr char kGripperGroup[] = "gripper";
constexpr char kEndEffector[] = "gripper";
constexpr char kTcpFrame[] = "tcp_link";
constexpr char kOpenPose[] = "open";
constexpr char kClosedPose[] = "closed";
constexpr char kWorkPose[] = "work";
constexpr char kGroundId[] = "ground";
// The observed support also exists as occupied cells, not only the rigid plane.
constexpr std::array<const char *, 2> kSupportIds{kGroundId, "<octomap>"};
constexpr std::array<const char *, 2> kFingerLinks{"link7_left", "link7_right"};
constexpr double kGroundSize = 2.0;
constexpr double kGroundDepth = 1.0;
constexpr double kDefaultVelocityScaling = 0.125; // User halved grasp/place speed; physical limits unchanged.
constexpr double kClosingVelocityScaling = kDefaultVelocityScaling / 2.0;

inline Eigen::Isometry3d pose_transform(const geometry_msgs::msg::Pose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                                     pose.orientation.y, pose.orientation.z).toRotationMatrix();
  return result;
}


} // namespace stararm_mtc
