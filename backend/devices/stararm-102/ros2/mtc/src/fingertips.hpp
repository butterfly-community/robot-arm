#pragma once

#include <moveit/robot_state/robot_state.hpp>
#include <geometric_shapes/shapes.h>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace stararm {
using FingertipPair = std::array<Eigen::Vector3d, 2>;

// A POINT attached to a moving finger, not a finger axis, pad plane, TCP or
// GraspGenX's sampling-depth reference. This device's audited CAD fingers extend
// along link-local +X. Their distal-most face/edge centre defines the tip.
// Extract from the loaded collision mesh (same STL as visual geometry), so no
// independently maintained coordinates, guessed offsets or new collision body.
struct FingertipReference {
  std::string link;
  Eigen::Vector3d point;

  Eigen::Vector3d position(const moveit::core::RobotState &state) const {
    return state.getGlobalLinkTransform(link) * point;
  }
};

inline FingertipReference fingertip_reference(const moveit::core::RobotModel &model,
                                             const std::string &name) {
  const auto *link = model.getLinkModel(name);
  if (!link) throw std::runtime_error("Missing fingertip parent: " + name);
  std::vector<Eigen::Vector3d> vertices;
  const auto &shapes = link->getShapes();
  const auto &origins = link->getCollisionOriginTransforms();
  for (std::size_t i = 0; i < shapes.size(); ++i) {
    if (shapes[i]->type != shapes::MESH) continue;
    const auto &mesh = static_cast<const shapes::Mesh &>(*shapes[i]);
    for (unsigned int j = 0; j < mesh.vertex_count; ++j)
      vertices.push_back(origins[i] * Eigen::Vector3d(mesh.vertices[3*j],
                           mesh.vertices[3*j+1], mesh.vertices[3*j+2]));
  }
  if (vertices.empty()) throw std::runtime_error("Missing fingertip CAD mesh: " + name);
  double end = -std::numeric_limits<double>::infinity();
  for (const auto &v : vertices) end = std::max(end, v.x());
  Eigen::Vector3d low = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d high = -low;
  for (const auto &v : vertices) if (v.x() == end) {
    low = low.cwiseMin(v);
    high = high.cwiseMax(v);
  }
  return {name, (low + high) / 2.0};
}

inline std::array<FingertipReference, 2> fingertip_references(
    const moveit::core::RobotModel &model) {
  return {fingertip_reference(model, "link7_left"),
          fingertip_reference(model, "link7_right")};
}

inline FingertipPair fingertip_positions(const std::array<FingertipReference, 2> &references,
                                        const moveit::core::RobotState &state) {
  return {references[0].position(state), references[1].position(state)};
}

// Evaluate the open hand used at the end of approach. Actual force-controlled
// contact aperture is not known during planning; do not claim contact geometry.
inline FingertipPair open_fingertips_tcp(const moveit::core::RobotModelConstPtr &model) {
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.setToDefaultValues(model->getJointModelGroup("gripper"), "open");
  state.update();
  auto tips = fingertip_positions(fingertip_references(*model), state);
  const auto from_base = state.getGlobalLinkTransform("tcp_link").inverse();
  for (auto &point : tips) point = from_base * point;
  return tips;
}

// Both points must be in the task-ground frame: ground(goal.frame_id) has
// normal +Z. This is the line-to-plane angle, not a body or camera angle.
inline double fingertip_level_cost(const FingertipPair &tips) {
  const Eigen::Vector3d difference = tips[1] - tips[0];
  return difference.norm() == 0.0 ? 0.0 : std::abs(difference.z()) / difference.norm();
}

// User-selected limit: reject a two-tip line at or above 20 degrees to ground.
// This does not constrain either finger/body axis or the opening plane.
inline constexpr double kMaximumFingertipTiltDegrees = 20.0;
inline bool fingertip_tilt_allowed(const FingertipPair &tips) {
  return fingertip_level_cost(tips) <
      std::sin(kMaximumFingertipTiltDegrees * M_PI / 180.0);
}
}  // namespace stararm
