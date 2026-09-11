#pragma once
#include <fcl/fcl.h>
#include <geometric_shapes/shapes.h>

// Self-filter only: a point is within the configured Euclidean distance of
// the observed robot mesh iff a sphere at that point intersects its surface.
// FCL supplies BVH traversal and triangle/sphere tests; no custom distance math.
// Official ShapeMask still handles solid interiors and sensor shadow rays.
class MeshSurfaceMask {
public:
  MeshSurfaceMask(const shapes::Mesh& mesh, double padding)
      : sphere_(padding), padding_(padding) {
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;
    bounds_.setEmpty();
    vertices.reserve(mesh.vertex_count);
    for (unsigned i = 0; i < mesh.vertex_count; ++i) {
      vertices.emplace_back(mesh.vertices[3*i], mesh.vertices[3*i+1], mesh.vertices[3*i+2]);
      bounds_.extend(vertices.back());
    }
    triangles.reserve(mesh.triangle_count);
    for (unsigned i = 0; i < mesh.triangle_count; ++i)
      triangles.emplace_back(mesh.triangles[3*i], mesh.triangles[3*i+1], mesh.triangles[3*i+2]);
    bounds_.min().array() -= padding;
    bounds_.max().array() += padding;
    model_.beginModel();
    model_.addSubModel(vertices, triangles);
    model_.endModel();
  }

  bool contains(const Eigen::Vector3d& point) const {
    if (padding_ <= 0 || !point.allFinite() || !bounds_.contains(point)) return false;
    fcl::Transform3d pose = fcl::Transform3d::Identity();
    pose.translation() = point;
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    fcl::collide(&model_, fcl::Transform3d::Identity(), &sphere_, pose, request, result);
    return result.isCollision();
  }

  // Swept sensor ray through the SAME measured mesh uncertainty envelope.
  // FCL capsule/BVH distance geometry avoids radial vertex inflation, which
  // underestimates the side of long thin fingers. No added clearance value.
  bool intersectsSegment(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const {
    if (padding_ <= 0 || !from.allFinite() || !to.allFinite()) return false;
    Eigen::AlignedBox3d segment_bounds;
    segment_bounds.setEmpty();
    segment_bounds.extend(from); segment_bounds.extend(to);
    if (!bounds_.intersects(segment_bounds)) return false;
    const Eigen::Vector3d delta = to - from;
    const double length = delta.norm();
    if (length == 0) return contains(from);
    fcl::Capsuled ray(padding_, length);
    fcl::Transform3d pose = fcl::Transform3d::Identity();
    pose.translation() = (from + to) / 2;
    pose.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), delta / length).toRotationMatrix();
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    fcl::collide(&model_, fcl::Transform3d::Identity(), &ray, pose, request, result);
    return result.isCollision();
  }

private:
  fcl::BVHModel<fcl::OBBRSSd> model_;
  fcl::Sphered sphere_;
  double padding_;
  Eigen::AlignedBox3d bounds_;
};
