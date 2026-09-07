// Use the installed official updater for masks, ray tracing and Octomap.
// Only the excluded mesh envelope differs: the StarArm has long, thin meshes
// whose radial padding is much smaller along their surface normals. Applying
// that mesh margin globally also erased the observed 3 mm acrylic sheet via
// ground-box padding. Primitive/world-object padding therefore stays separate.
#include <geometric_shapes/body_operations.h>
#include <moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace stararm_102 {
class MeshSelfFilter final : public occupancy_map_monitor::PointCloudOctomapUpdater {
public:
  bool initialize(const rclcpp::Node::SharedPtr& node) override {
    node_ = node;
    return PointCloudOctomapUpdater::initialize(node);
  }

  bool setParams(const std::string& ns) override {
    if (!PointCloudOctomapUpdater::setParams(ns)) return false;
    double common_padding;
    double mesh_padding;
    if (!node_->get_parameter(ns + ".padding_offset", common_padding) ||
        !node_->get_parameter(ns + ".mesh_padding_offset", mesh_padding)) return false;
    extra_mesh_padding_ = mesh_padding - common_padding;
    return true;
  }

  occupancy_map_monitor::ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape) override {
    if (shape->type != shapes::MESH || extra_mesh_padding_ == 0.0)
      return PointCloudOctomapUpdater::excludeShape(shape);
    std::unique_ptr<bodies::Body> envelope(bodies::createBodyFromShape(shape.get()));
    envelope->setPadding(extra_mesh_padding_);
    return PointCloudOctomapUpdater::excludeShape(bodies::constructShapeFromBody(envelope.get()));
  }

private:
  rclcpp::Node::SharedPtr node_;
  double extra_mesh_padding_ = 0.0;
};
}  // namespace stararm_102
PLUGINLIB_EXPORT_CLASS(stararm_102::MeshSelfFilter, occupancy_map_monitor::OccupancyMapUpdater)
