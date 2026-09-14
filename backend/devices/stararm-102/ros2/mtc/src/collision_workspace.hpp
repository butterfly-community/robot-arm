#pragma once
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <moveit/robot_model/robot_model.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <map>

// Triangle-inequality envelope of ALL robot collision geometry over all rotations.
// Not a joint limit or a task workspace restriction. Outside this envelope a
// measured obstacle cannot touch this fixed-base robot in any configuration.
inline double collision_reach(const moveit::core::RobotModel& model,
                              const moveit::core::LinkModel* attachment = nullptr,
                              double attached_extent = 0.0) {
  std::map<const moveit::core::LinkModel*, double> origins;
  double radius = 0.0;
  for (const auto* link : model.getLinkModels()) {
    const auto* parent = link->getParentLinkModel();
    double origin = parent ? origins.at(parent) : 0.0;
    origin += link->getJointOriginTransform().translation().norm();
    const auto* joint = link->getParentJointModel();
    if (joint && joint->getType() == moveit::core::JointModel::PRISMATIC) {
      const auto& bound = joint->getVariableBounds().front();
      origin += std::max(std::abs(bound.min_position_), std::abs(bound.max_position_));
    }
    origins[link] = origin;
    radius = std::max(radius, origin);
    // Held geometry extends from its attachment frame, not from the farthest
    // robot collision surface (which already includes the fingers).
    if (link == attachment)
      radius = std::max(radius, origin + attached_extent);
    for (std::size_t i = 0; i < link->getShapes().size(); ++i) {
      std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(link->getShapes()[i].get()));
      bodies::BoundingSphere sphere;
      body->computeBoundingSphere(sphere);
      radius = std::max(radius, origin + link->getCollisionOriginTransforms()[i].translation().norm()
                                      + sphere.center.norm() + sphere.radius);
    }
  }
  return radius;
}

// Filter membership in the fixed model frame; output coordinates, fields and
// sensor origin are unchanged, so the official updater still performs ray casts.
inline sensor_msgs::msg::PointCloud2 collision_workspace_cloud(
    const sensor_msgs::msg::PointCloud2& input, const Eigen::Isometry3d& sensor_in_model,
    double radius) {
  auto result = input;
  result.data.clear();
  result.data.reserve(input.data.size());
  sensor_msgs::PointCloud2ConstIterator<float> x(input, "x"), y(input, "y"), z(input, "z");
  for (std::size_t i = 0; i < std::size_t(input.width) * input.height; ++i, ++x, ++y, ++z) {
    const auto position = sensor_in_model * Eigen::Vector3d(*x, *y, *z);
    if (position.squaredNorm() <= radius * radius) {
      const auto begin = input.data.begin() + i * input.point_step;
      result.data.insert(result.data.end(), begin, begin + input.point_step);
    }
  }
  result.height = 1;
  result.width = result.data.size() / input.point_step;
  result.row_step = result.width * result.point_step;
  return result;
}
