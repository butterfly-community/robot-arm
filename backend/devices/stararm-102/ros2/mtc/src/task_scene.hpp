#pragma once
#include "grasp_candidates.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <condition_variable>
#include <mutex>

namespace stararm_mtc {
inline moveit_msgs::msg::CollisionObject box(const std::string &frame_id,
                                      const std::string &id,
                                      const geometry_msgs::msg::Pose &pose,
                                      const geometry_msgs::msg::Vector3 &size) {
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame_id;
  object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {size.x, size.y, size.z};
  object.primitives.push_back(std::move(primitive));
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

inline moveit_msgs::msg::CollisionObject ground(const std::string &frame_id) {
  geometry_msgs::msg::Pose pose;
  pose.position.z = -kGroundDepth / 2.0;
  pose.orientation.w = 1.0;
  geometry_msgs::msg::Vector3 size;
  size.x = kGroundSize;
  size.y = kGroundSize;
  size.z = kGroundDepth;
  return box(frame_id, kGroundId, pose, size);
}

// Owns the frozen task map and matching updater acknowledgement. No planning or execution.
class TaskScene {
public:
  TaskScene(rclcpp::Node &node, double resolution) : node_(node), octomap_resolution_(resolution) {
    cloud_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/depth/points", rclcpp::SensorDataQoS());
    filtered_subscription_ = node_.create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/depth/filtered", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud) {
          std::lock_guard<std::mutex> lock(cloud_mutex_);
          filtered_stamp_ = cloud->header.stamp;
          cloud_ready_.notify_all();
        });
    clear_octomap_ = node_.create_client<std_srvs::srv::Empty>("/clear_octomap");
    sensor_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(&node_);
  }

  void apply(const PickPlace::Goal &goal, const moveit::core::RobotModelConstPtr& model) {
    const auto begun = std::chrono::steady_clock::now();
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(ground(goal.frame_id));
    objects.push_back(
        box(goal.frame_id, goal.object_id, goal.object_pose, goal.object_size));
    const auto actual_target = objects.back();
    // RGB-D edge ramps can leave target-owned occupied cells outside the
    // segmented box. Exclude its adjacent voxel layer during native filtering
    // only. Restore the actual collision/attachment dimensions before planning.
    // Real replay: 17/121 closure collisions -> 0; all 707 destination cells
    // unchanged. Do not enlarge the ground or relax collision checking.
    for (auto &dimension : objects.back().primitives.front().dimensions)
      dimension += 2.0 * octomap_resolution_;
    if (!scene_.applyCollisionObjects(objects)) {
      throw std::runtime_error("MoveIt rejected the task planning scene");
    }
    // Establish the independent target before the native updater self-filters
    // its duplicate points. One observation builds this task's frozen map.
    auto cleared = clear_octomap_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
    cleared.get();
    Eigen::Isometry3d sensor_in_scene = Eigen::Isometry3d::Identity();
    const auto& sensor = goal.sensor_in_scene;
    sensor_in_scene.translation() = Eigen::Vector3d(sensor.position.x, sensor.position.y, sensor.position.z);
    sensor_in_scene.linear() = Eigen::Quaterniond(sensor.orientation.w, sensor.orientation.x,
        sensor.orientation.y, sensor.orientation.z).toRotationMatrix();
    moveit::core::RobotState reference(model);
    reference.setToDefaultValues();
    reference.update();
    const auto scene_in_model = goal.frame_id == model->getModelFrame()
        ? Eigen::Isometry3d::Identity() : reference.getGlobalLinkTransform(goal.frame_id);
    const double radius = collision_workspace_radius(*model, goal, octomap_resolution_);
    auto cloud = collision_workspace_cloud(goal.scene_cloud, scene_in_model * sensor_in_scene, radius);
    RCLCPP_INFO(node_.get_logger(), "collision workspace radius %.6f m: %u -> %u points",
                radius, goal.scene_cloud.width * goal.scene_cloud.height, cloud.width * cloud.height);
    cloud.header.stamp = node_.now();
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = goal.frame_id;
    transform.header.stamp = cloud.header.stamp;
    transform.child_frame_id = cloud.header.frame_id;
    transform.transform.translation.x = goal.sensor_in_scene.position.x;
    transform.transform.translation.y = goal.sensor_in_scene.position.y;
    transform.transform.translation.z = goal.sensor_in_scene.position.z;
    transform.transform.rotation = goal.sensor_in_scene.orientation;
    sensor_tf_->sendTransform(transform);
    const auto prepared = std::chrono::steady_clock::now();
    cloud_publisher_->publish(cloud);
    std::unique_lock<std::mutex> lock(cloud_mutex_);
    // Only the corresponding callback certifies that Octomap finished updating.
    // Do not mistake publishing the message for a completed scene update.
    cloud_ready_.wait(lock, [&] { return filtered_stamp_ == cloud.header.stamp; });
    lock.unlock();
    if (!scene_.applyCollisionObject(actual_target))
      throw std::runtime_error("MoveIt rejected restoration of actual target dimensions");
    RCLCPP_INFO(node_.get_logger(), "request %s scene: prepare %.6f s, update/sync %.6f s", goal.request_id.c_str(),
        std::chrono::duration<double>(prepared-begun).count(),
        std::chrono::duration<double>(std::chrono::steady_clock::now()-prepared).count());
  }

  void cleanup(const std::vector<std::string> &temporary_ids) {
    auto attached_objects = scene_.getAttachedObjects(temporary_ids);
    std::vector<moveit_msgs::msg::AttachedCollisionObject> removals;
    removals.reserve(attached_objects.size());
    for (auto &entry : attached_objects) {
      auto &attached = entry.second;
      attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      removals.push_back(std::move(attached));
    }
    if (!removals.empty()) {
      scene_.applyAttachedCollisionObjects(removals);
    }
    scene_.removeCollisionObjects(temporary_ids);
    // This map describes the pre-grasp observation, not the scene after release.
    // Keep it for the complete task, then discard it with the temporary target.
    clear_octomap_->async_send_request(
        std::make_shared<std_srvs::srv::Empty::Request>()).get();
  }

private:
  rclcpp::Node &node_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  double octomap_resolution_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_subscription_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> sensor_tf_;
  std::mutex cloud_mutex_;
  std::condition_variable cloud_ready_;
  builtin_interfaces::msg::Time filtered_stamp_;
};

} // namespace stararm_mtc
