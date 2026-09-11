#pragma once

#include "mesh_surface_mask.hpp"

#include <geometric_shapes/body_operations.h>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <memory>
#include <stdexcept>
#include <vector>

namespace stararm {

// Geometric visibility is only a choice of observation waypoint, not a new
// collision allowance or a substitute for the subsequent real RGB-D frame.
// Use geometric_shapes ray queries (also used by MoveIt's self-filter), with
// the deployed link shapes and FK. No camera-specific retreat distance.
class ObservationVisibility {
public:
  explicit ObservationVisibility(const moveit::core::RobotModel& model, double mesh_uncertainty) {
    for (const auto* link : model.getLinkModelsWithCollisionGeometry())
      for (std::size_t i = 0; i < link->getShapes().size(); ++i) {
        const auto* shape = link->getShapes()[i].get();
        bodies_.push_back({link, i, std::unique_ptr<bodies::Body>(
            bodies::createBodyFromShape(shape)),
            shape->type == shapes::MESH && mesh_uncertainty > 0
              ? std::make_unique<MeshSurfaceMask>(*static_cast<const shapes::Mesh*>(shape), mesh_uncertainty)
              : nullptr, Eigen::Isometry3d::Identity()});
      }
  }

  std::size_t visible(const moveit::core::RobotState& state,
                      const Eigen::Vector3d& camera,
                      const EigenSTL::vector_Vector3d& points) {
    for (auto& body : bodies_) {
      const auto& pose = state.getCollisionBodyTransform(body.link, body.index);
      body.shape->setPose(pose);
      body.mesh_from_world = pose.inverse();
    }
    std::size_t count = 0;
    for (const auto& point : points) {
      const Eigen::Vector3d ray = point - camera;
      const auto distance = ray.norm();
      bool blocked = false;
      for (const auto& body : bodies_) {
        EigenSTL::vector_Vector3d hits;
        if ((body.uncertainty && body.uncertainty->intersectsSegment(
                 body.mesh_from_world * camera, body.mesh_from_world * point)) ||
            body.shape->containsPoint(camera) ||
            (body.shape->intersectsRay(camera, ray / distance, &hits, 1) &&
             !hits.empty() && (hits.front() - camera).norm() < distance)) {
          blocked = true;
          break;
        }
      }
      count += !blocked;
    }
    return count;
  }

  std::pair<std::size_t, std::size_t> waypoint(
      const robot_trajectory::RobotTrajectory& path,
      const Eigen::Vector3d& camera, const EigenSTL::vector_Vector3d& points) {
    if (path.empty() || points.empty())
      throw std::runtime_error("Observation needs a trajectory and observed target points");
    std::pair<std::size_t, std::size_t> best{path.getWayPointCount()-1, 0};
    // Latest maximally visible waypoint: move towards the object but stop
    // before occluding it. If none sees the full target, choose the best view
    // on this path, not an invented visibility threshold or an old image.
    for (std::size_t i = path.getWayPointCount(); i-- > 0;) {
      const auto count = visible(path.getWayPoint(i), camera, points);
      if (count > best.second) best = {i, count};
      if (count == points.size()) return {i, count};
    }
    return best;
  }

private:
  struct LinkBody {
    const moveit::core::LinkModel* link;
    std::size_t index;
    std::unique_ptr<bodies::Body> shape;
    std::unique_ptr<MeshSurfaceMask> uncertainty;
    Eigen::Isometry3d mesh_from_world;
  };
  std::vector<LinkBody> bodies_;
};

} // namespace stararm
