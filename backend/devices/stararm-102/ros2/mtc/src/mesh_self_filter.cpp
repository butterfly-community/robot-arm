// Use the installed official updater for masks, ray tracing and Octomap.
// Robot mesh uncertainty is a Euclidean surface distance, not radial vertex
// inflation (which under-filters thin links). Primitive/ground masks stay native.
#include "mesh_surface_mask.hpp"
#include <moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <map>
#include <mutex>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace stararm_102 {
class MeshSelfFilter final : public occupancy_map_monitor::PointCloudOctomapUpdater {
public:
  bool initialize(const rclcpp::Node::SharedPtr& node) override {
    node_ = node;
    return PointCloudOctomapUpdater::initialize(node);
  }

  bool setParams(const std::string& ns) override {
    if (!PointCloudOctomapUpdater::setParams(ns)) return false;
    return node_->get_parameter(ns + ".mesh_padding_offset", mesh_padding_) &&
           node_->get_parameter(ns + ".padding_scale", scale_);
  }

  occupancy_map_monitor::ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape) override {
    auto handle = PointCloudOctomapUpdater::excludeShape(shape);
    if (shape->type == shapes::MESH && mesh_padding_ > 0) {
      std::unique_ptr<shapes::Shape> scaled(shape->clone());
      scaled->scaleAndPadd(scale_, 0.0);
      auto surface = std::make_shared<MeshSurfaceMask>(
          static_cast<const shapes::Mesh&>(*scaled), mesh_padding_);
      std::lock_guard<std::mutex> lock(mesh_mutex_);
      meshes_.emplace(handle, std::move(surface));
    }
    return handle;
  }

  void forgetShape(occupancy_map_monitor::ShapeHandle handle) override {
    PointCloudOctomapUpdater::forgetShape(handle);
    std::lock_guard<std::mutex> lock(mesh_mutex_);
    meshes_.erase(handle);
  }

protected:
  void updateMask(const sensor_msgs::msg::PointCloud2& cloud,
                  const Eigen::Vector3d&, std::vector<int>& mask) override {
    std::vector<std::pair<std::shared_ptr<MeshSurfaceMask>, Eigen::Isometry3d>> surfaces;
    {
      std::lock_guard<std::mutex> lock(mesh_mutex_);
      for (const auto& [handle, mesh] : meshes_) {
        const auto transform = transform_cache_.find(handle);
        if (transform != transform_cache_.end())
          surfaces.emplace_back(mesh, transform->second.inverse());
      }
    }
    // FCL queries own their result buffers; the mesh and transforms are read-only.
    // Disjoint output ranges preserve the exact sequential mask, without sampling.
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, mask.size()), [&](const auto& range) {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      x += range.begin(); y += range.begin(); z += range.begin();
      for (std::size_t i = range.begin(); i < range.end(); ++i, ++x, ++y, ++z) {
        if (mask[i] != point_containment_filter::ShapeMask::OUTSIDE) continue;
        const Eigen::Vector3d point(*x, *y, *z);
        for (const auto& [mesh, cloud_in_mesh] : surfaces) {
          if (mesh->contains(cloud_in_mesh * point)) {
            mask[i] = point_containment_filter::ShapeMask::INSIDE;
            break;
          }
        }
      }
    });
  }

private:
  rclcpp::Node::SharedPtr node_;
  double mesh_padding_ = 0.0;
  double scale_ = 1.0;
  std::mutex mesh_mutex_;
  std::map<occupancy_map_monitor::ShapeHandle, std::shared_ptr<MeshSurfaceMask>> meshes_;
};
}  // namespace stararm_102
PLUGINLIB_EXPORT_CLASS(stararm_102::MeshSelfFilter, occupancy_map_monitor::OccupancyMapUpdater)
