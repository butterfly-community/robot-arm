#include <Eigen/Geometry>

#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "stararm_102_mtc/action/pick_place.hpp"

namespace mtc = moveit::task_constructor;
using PickPlace = stararm_102_mtc::action::PickPlace;
using GoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;

namespace {
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
constexpr double kDefaultVelocityScaling = 0.25; // User default; physical limits unchanged.

// Upstream checks coarse joint interpolation BEFORE time parameterization.
// Retiming introduces different waypoints: real replay had a narrow collision
// at 27..31 degrees, missed by the 0.1-rad grid but hit by retimed point 6.
// Check the exact outgoing path now, with the same native scene/ACM, instead of
// discovering that invalid waypoint only after the arm reaches the object.
class CheckedJointInterpolation final : public mtc::solvers::JointInterpolationPlanner {
public:
  using JointInterpolationPlanner::plan;
  Result plan(const planning_scene::PlanningSceneConstPtr &from,
              const planning_scene::PlanningSceneConstPtr &to,
              const moveit::core::JointModelGroup *group, double timeout,
              robot_trajectory::RobotTrajectoryPtr &result,
              const moveit_msgs::msg::Constraints &constraints) override {
    auto status = JointInterpolationPlanner::plan(from, to, group, timeout, result, constraints);
    if (status && !from->isPathValid(*result, constraints, group->getName()))
      return {false, "Retimed joint trajectory is invalid in the planning scene"};
    return status;
  }
};

// Lift along the requested line with the grasp orientation unchanged. A free
// position-only endpoint allowed a 124-degree wrist turn during a 20-mm lift.
// Keep the measured support-contact handling, using MoveIt's native Cartesian IK.
class SupportAwareCartesianPath final : public mtc::solvers::CartesianPath {
public:
  SupportAwareCartesianPath() {
    setMaxVelocityScalingFactor(kDefaultVelocityScaling);
  }
  using CartesianPath::plan;

  Result plan(const planning_scene::PlanningSceneConstPtr &from,
              const moveit::core::LinkModel &link,
              const Eigen::Isometry3d &offset, const Eigen::Isometry3d &target,
              const moveit::core::JointModelGroup *group, double timeout,
              robot_trajectory::RobotTrajectoryPtr &result,
              const moveit_msgs::msg::Constraints &path_constraints) override {
    auto planning_scene = from->diff();
    std::vector<const moveit::core::AttachedBody *> bodies;
    from->getCurrentState().getAttachedBodies(bodies);
    for (const auto *body : bodies) {
      for (const auto *support_id : kSupportIds) {
        auto &acm = planning_scene->getAllowedCollisionMatrixNonConst();
        collision_detection::AllowedCollision::Type allowed;
        if (!acm.getAllowedCollision(body->getName(), support_id, allowed) ||
            allowed != collision_detection::AllowedCollision::ALWAYS) {
          continue;
        }
        // Support contact during lift is not permission to drive into support.
        // Preserve only the contact already present in the observed geometry.
        // Native conditional ACM remains local to planning, not a ROS message.
        acm.setEntry(body->getName(), support_id, false);
        collision_detection::CollisionRequest request;
        request.contacts = true;
        request.max_contacts = std::numeric_limits<std::size_t>::max();
        request.max_contacts_per_pair = request.max_contacts;
        collision_detection::CollisionResult contacts;
        planning_scene->checkCollision(request, contacts);
        double initial_depth = 0.0;
        for (const auto &[pair, entries] : contacts.contacts) {
          if ((pair.first == body->getName() && pair.second == support_id) ||
              (pair.second == body->getName() && pair.first == support_id)) {
            for (const auto &contact : entries)
              initial_depth = std::max(initial_depth, contact.depth);
          }
        }
        collision_detection::DecideContactFn contact_allowed =
            [initial_depth](collision_detection::Contact &contact) {
              return contact.depth <= initial_depth;
            };
        acm.setEntry(body->getName(), support_id, contact_allowed);
      }
    }
    return CartesianPath::plan(planning_scene, link, offset, target, group,
                               timeout, result, path_constraints);
  }
};

// The release endpoint remains position-only as requested by the user. This
// freedom belongs to transport, not the short lift while still at the object.
class PositionOnlyPlanner final : public mtc::solvers::PipelinePlanner {
public:
  explicit PositionOnlyPlanner(const rclcpp::Node::SharedPtr &node) : PipelinePlanner(node) {
    setMaxVelocityScalingFactor(kDefaultVelocityScaling);
  }
  using PipelinePlanner::plan;

  Result plan(const planning_scene::PlanningSceneConstPtr &from,
              const moveit::core::LinkModel &link,
              const Eigen::Isometry3d &offset, const Eigen::Isometry3d &target,
              const moveit::core::JointModelGroup *group, double timeout,
              robot_trajectory::RobotTrajectoryPtr &result,
              const moveit_msgs::msg::Constraints &path_constraints) override {
    const double tolerance =
        properties().get<double>("goal_position_tolerance");
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = from->getPlanningFrame();
    point.point.x = target.translation().x();
    point.point.y = target.translation().y();
    // The lower edge, not the centre, must meet the requested height.
    point.point.z = target.translation().z() + tolerance;
    auto goal = kinematic_constraints::constructGoalConstraints(link.getName(), point, tolerance);
    auto &tcp_offset = goal.position_constraints.front().target_point_offset;
    tcp_offset.x = offset.translation().x();
    tcp_offset.y = offset.translation().y();
    tcp_offset.z = offset.translation().z();
    return PipelinePlanner::plan(from, group, goal, timeout, result, path_constraints);
  }
};

struct DepthPose {
  geometry_msgs::msg::PoseStamped pose;
  std::size_t candidate_index;
  double depth_m;
};

// Search only the one-dimensional insertion coordinate, not a new orientation
// or a model-side offset. Bound insertion by the observed object's far face
// projected onto TCP +Z. Use the existing scene resolution, including both
// endpoints; this is a discrete search, not a claim of an exact global optimum.
std::vector<DepthPose> grasp_depth_poses(const PickPlace::Goal &goal, double resolution) {
  std::vector<DepthPose> result;
  const auto &object = goal.object_pose;
  const Eigen::Vector3d center(object.position.x, object.position.y, object.position.z);
  const Eigen::Quaterniond object_rotation(object.orientation.w, object.orientation.x,
                                          object.orientation.y, object.orientation.z);
  const Eigen::Vector3d size(goal.object_size.x, goal.object_size.y, goal.object_size.z);
  for (std::size_t index = 0; index < goal.grasp_poses.size(); ++index) {
    const auto &pose = goal.grasp_poses[index];
    const Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x,
                                      pose.orientation.y, pose.orientation.z);
    const Eigen::Vector3d axis = rotation * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
    const Eigen::Vector3d object_axis = object_rotation.conjugate() * axis;
    const double far_face = axis.dot(center - position) + 0.5 * object_axis.cwiseAbs().dot(size);
    const double maximum = std::max(0.0, far_face);
    const std::size_t steps = static_cast<std::size_t>(std::ceil(maximum / resolution));
    for (std::size_t step = 0; step <= steps; ++step) {
      const double depth = std::min(step * resolution, maximum);
      DepthPose variant;
      variant.pose.header.frame_id = goal.frame_id;
      variant.pose.pose = pose;
      variant.pose.pose.position.x += axis.x() * depth;
      variant.pose.pose.position.y += axis.y() * depth;
      variant.pose.pose.position.z += axis.z() * depth;
      variant.candidate_index = index;
      variant.depth_m = depth;
      result.push_back(std::move(variant));
    }
  }
  return result;
}

class GeneratePoses final : public mtc::stages::GeneratePose {
public:
  GeneratePoses(const std::string &name,
                std::vector<DepthPose> poses)
      : mtc::stages::GeneratePose(name),
        poses_(std::move(poses)) {}

  void compute() override {
    if (upstream_solutions_.empty()) {
      return;
    }
    const auto &upstream = *upstream_solutions_.pop();
    const auto scene = upstream.end()->scene()->diff();
    for (std::size_t index = 0; index < poses_.size(); ++index) {
      mtc::InterfaceState state(scene);
      forwardProperties(*upstream.end(), state);
      const auto &variant = poses_[index];
      state.properties().set("target_pose", variant.pose);
      state.properties().set("grasp_candidate_index", variant.candidate_index);
      state.properties().set("grasp_depth_m", variant.depth_m);
      mtc::SubTrajectory trajectory;
      trajectory.setComment("candidate " + std::to_string(variant.candidate_index) +
                            " depth +" + std::to_string(variant.depth_m * 1000.0) + " mm");
      spawn(std::move(state), std::move(trajectory));
    }
  }

private:
  std::vector<DepthPose> poses_;
};

// Scene supplies candidates in descending model confidence. Preserve that
// ordering among COMPLETE plans, then prefer deeper insertion within that
// candidate, and finally motion cost. A zero-depth variant is always retained.
// Joint travel alone is not a measure of whether a learned grasp will hold.
std::size_t grasp_candidate_index(const mtc::SolutionBase &solution) {
  // ComputeIK creates a NEW SubTrajectory, not a wrapper around the generator.
  // Read the explicitly forwarded interface property instead of its creator.
  if (solution.end() && solution.end()->properties().hasProperty("grasp_candidate_index"))
    return solution.end()->properties().get<std::size_t>("grasp_candidate_index");
  if (const auto *wrapped = dynamic_cast<const mtc::WrappedSolution *>(&solution))
    return grasp_candidate_index(*wrapped->wrapped());
  std::size_t index = std::numeric_limits<std::size_t>::max();
  if (const auto *sequence = dynamic_cast<const mtc::SolutionSequence *>(&solution))
    for (const auto *child : sequence->solutions())
      index = std::min(index, grasp_candidate_index(*child));
  return index;
}

double grasp_depth(const mtc::SolutionBase &solution) {
  if (solution.end() && solution.end()->properties().hasProperty("grasp_depth_m"))
    return solution.end()->properties().get<double>("grasp_depth_m");
  if (const auto *wrapped = dynamic_cast<const mtc::WrappedSolution *>(&solution))
    return grasp_depth(*wrapped->wrapped());
  double depth = 0.0;
  if (const auto *sequence = dynamic_cast<const mtc::SolutionSequence *>(&solution))
    for (const auto *child : sequence->solutions())
      depth = std::max(depth, grasp_depth(*child));
  return depth;
}

auto grasp_solution_rank(const mtc::SolutionBase &solution) {
  return std::make_tuple(grasp_candidate_index(solution), -grasp_depth(solution), solution.cost());
}

moveit_msgs::msg::CollisionObject box(const std::string &frame_id,
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

moveit_msgs::msg::CollisionObject ground(const std::string &frame_id) {
  geometry_msgs::msg::Pose pose;
  pose.position.z = -kGroundDepth / 2.0;
  pose.orientation.w = 1.0;
  geometry_msgs::msg::Vector3 size;
  size.x = kGroundSize;
  size.y = kGroundSize;
  size.z = kGroundDepth;
  return box(frame_id, kGroundId, pose, size);
}

geometry_msgs::msg::Vector3Stamped direction(const std::string &frame_id,
                                             double z) {
  geometry_msgs::msg::Vector3Stamped result;
  result.header.frame_id = frame_id;
  result.vector.z = z;
  return result;
}

std::string failure_summary(const mtc::Task &task,
                            const std::string &stage_name) {
  std::map<std::string, std::size_t> counts;
  task.stages()->traverseRecursively(
      [&](const mtc::Stage &stage, unsigned int) {
        if (stage.name() == stage_name) {
          for (const auto &failure : stage.failures()) {
            auto reason = failure->comment();
            if (reason.rfind("candidate ", 0) == 0) {
              const auto separator = reason.find(": ");
              if (separator != std::string::npos) {
                reason.erase(0, separator + 2);
              }
            }
            ++counts[reason];
          }
        }
        return true;
      });
  std::ostringstream summary;
  for (const auto &[reason, count] : counts) {
    summary << (summary.tellp() == 0 ? "" : "; ") << count << "x "
            << (reason.empty() ? "unspecified failure" : reason);
  }
  return summary.str();
}

} // namespace

class PickPlaceServer : public rclcpp::Node {
public:
  PickPlaceServer() : Node("stararm_102_mtc") {
    octomap_resolution_ = declare_parameter<double>("octomap_resolution");
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/depth/points", rclcpp::SensorDataQoS());
    filtered_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/depth/filtered", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud) {
          std::lock_guard<std::mutex> lock(cloud_mutex_);
          filtered_stamp_ = cloud->header.stamp;
          cloud_ready_.notify_all();
        });
    clear_octomap_ = create_client<std_srvs::srv::Empty>("/clear_octomap");
    sensor_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    server_ = rclcpp_action::create_server<PickPlace>(
        this, "/stararm102/pick_place",
        [](const rclcpp_action::GoalUUID &,
           std::shared_ptr<const PickPlace::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle>) {
          return rclcpp_action::CancelResponse::REJECT;
        },
        [this](const std::shared_ptr<GoalHandle> handle) {
          std::thread([this, handle] { execute(handle); }).detach();
        });
  }

private:
  void feedback(const std::shared_ptr<GoalHandle> &handle,
                const std::string &state, const std::string &stage,
                std::size_t solutions = 0, double cost = 0.0) {
    auto message = std::make_shared<PickPlace::Feedback>();
    message->state = state;
    message->stage = stage;
    message->solution_count = static_cast<std::uint32_t>(solutions);
    message->selected_cost = cost;
    handle->publish_feedback(message);
  }

  void apply_scene(const PickPlace::Goal &goal) {
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
    auto cloud = goal.scene_cloud;
    cloud.header.stamp = now();
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = goal.frame_id;
    transform.header.stamp = cloud.header.stamp;
    transform.child_frame_id = cloud.header.frame_id;
    transform.transform.translation.x = goal.sensor_in_scene.position.x;
    transform.transform.translation.y = goal.sensor_in_scene.position.y;
    transform.transform.translation.z = goal.sensor_in_scene.position.z;
    transform.transform.rotation = goal.sensor_in_scene.orientation;
    sensor_tf_->sendTransform(transform);
    cloud_publisher_->publish(cloud);
    std::unique_lock<std::mutex> lock(cloud_mutex_);
    // Only the corresponding callback certifies that Octomap finished updating.
    // Do not mistake publishing the message for a completed scene update.
    cloud_ready_.wait(lock, [&] { return filtered_stamp_ == cloud.header.stamp; });
    lock.unlock();
    if (!scene_.applyCollisionObject(actual_target))
      throw std::runtime_error("MoveIt rejected restoration of actual target dimensions");
  }

  void cleanup_scene(const std::vector<std::string> &temporary_ids) {
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

  mtc::Task create_task(const PickPlace::Goal &goal) {
    mtc::Task task;
    task.setName("pick and place " + goal.request_id);
    task.loadRobotModel(shared_from_this());
    task.setProperty("group", kArmGroup);
    task.setProperty("eef", kEndEffector);
    task.setProperty("hand", kGripperGroup);
    task.setProperty("ik_frame", kTcpFrame);

    auto pipeline =
        std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    auto joint_interpolation =
        std::make_shared<CheckedJointInterpolation>();
    pipeline->setMaxVelocityScalingFactor(kDefaultVelocityScaling);
    cartesian->setMaxVelocityScalingFactor(kDefaultVelocityScaling);
    joint_interpolation->setMaxVelocityScalingFactor(kDefaultVelocityScaling);

    auto current = std::make_unique<mtc::stages::CurrentState>("current state");
    task.add(std::move(current));

    {
      auto support = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow object support contact");
      for (const auto *support_id : kSupportIds)
        support->allowCollisions(goal.object_id, support_id, true);
      task.add(std::move(support));
    }

    mtc::Stage *open_stage = nullptr;
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper",
                                                         joint_interpolation);
      stage->setGroup(kGripperGroup);
      stage->setGoal(kOpenPose);
      open_stage = stage.get();
      task.add(std::move(stage));
    }
    {
      mtc::stages::Connect::GroupPlannerVector planners{{kArmGroup, pipeline}};
      auto stage =
          std::make_unique<mtc::stages::Connect>("move to pregrasp", planners);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage));
    }

    {
      auto pick = std::make_unique<mtc::SerialContainer>("pick object");
      task.properties().exposeTo(pick->properties(),
                                 {"eef", "hand", "group", "ik_frame"});
      pick->properties().configureInitFrom(
          mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

      auto approach = std::make_unique<mtc::stages::MoveRelative>(
          "approach object", cartesian);
      approach->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      approach->setIKFrame(kTcpFrame);
      approach->setMinMaxDistance(0.0, goal.object_size.z);
      approach->setDirection(direction(kTcpFrame, 1.0));
      pick->insert(std::move(approach));

      auto generator =
          std::make_unique<GeneratePoses>("grasp candidates", grasp_depth_poses(goal, octomap_resolution_));
      generator->properties().configureInitFrom(mtc::Stage::PARENT);
      generator->setMonitoredStage(open_stage);
      auto grasp_ik = std::make_unique<mtc::stages::ComputeIK>(
          "candidate IK", std::move(generator));
      grasp_ik->setGroup(kArmGroup);
      grasp_ik->setEndEffector(kEndEffector);
      grasp_ik->setMaxIKSolutions(8);
      grasp_ik->setForwardedProperties({"grasp_candidate_index", "grasp_depth_m"});
      grasp_ik->setIKFrame(Eigen::Isometry3d::Identity(), kTcpFrame);
      grasp_ik->properties().configureInitFrom(mtc::Stage::INTERFACE,
                                               {"target_pose"});
      pick->insert(std::move(grasp_ik));

      auto allow = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow gripper object collision");
      allow->allowCollisions(
          goal.object_id,
          std::vector<std::string>(kFingerLinks.begin(), kFingerLinks.end()),
          true);
      pick->insert(std::move(allow));

      auto close = std::make_unique<mtc::stages::MoveTo>("close gripper",
                                                         joint_interpolation);
      close->setGroup(kGripperGroup);
      close->setGoal(kClosedPose);
      pick->insert(std::move(close));

      auto attach =
          std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      attach->attachObject(goal.object_id, kTcpFrame);
      pick->insert(std::move(attach));

      auto lift =
          std::make_unique<mtc::stages::MoveRelative>(
              "lift object", std::make_shared<SupportAwareCartesianPath>());
      lift->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      lift->setIKFrame(kTcpFrame);
      // A zero minimum lets MoveRelative accept an empty Cartesian path.
      // Require the complete lift before restoring support collision.
      lift->setMinMaxDistance(goal.object_size.z, goal.object_size.z);
      lift->setDirection(direction(goal.frame_id, 1.0));
      pick->insert(std::move(lift));

      auto restore_support = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "restore support collision after lift");
      for (const auto *support_id : kSupportIds)
        restore_support->allowCollisions(goal.object_id, support_id, false);
      pick->insert(std::move(restore_support));

      task.add(std::move(pick));
    }

    {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task.properties().exposeTo(place->properties(),
                                 {"eef", "hand", "group", "ik_frame"});
      place->properties().configureInitFrom(
          mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

      geometry_msgs::msg::PointStamped release_point;
      release_point.header.frame_id = goal.frame_id;
      release_point.point = goal.placement_pose.position;
      auto release = std::make_unique<mtc::stages::MoveTo>(
          "transport to release point", std::make_shared<PositionOnlyPlanner>(shared_from_this()));
      release->setGroup(kArmGroup);
      release->setIKFrame(kTcpFrame);
      release->setGoal(release_point);
      place->insert(std::move(release));

      auto open = std::make_unique<mtc::stages::MoveTo>("open gripper",
                                                        joint_interpolation);
      open->setGroup(kGripperGroup);
      open->setGoal(kOpenPose);
      place->insert(std::move(open));

      auto detach =
          std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      detach->detachObject(goal.object_id, kTcpFrame);
      place->insert(std::move(detach));

      auto forbid = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "restore object collision");
      forbid->allowCollisions(
          goal.object_id,
          std::vector<std::string>(kFingerLinks.begin(), kFingerLinks.end()),
          false);
      place->insert(std::move(forbid));

      auto retreat = std::make_unique<mtc::stages::MoveRelative>(
          "retreat after place", cartesian);
      retreat->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      retreat->setIKFrame(kTcpFrame);
      retreat->setMinMaxDistance(0.0, goal.object_size.z);
      retreat->setDirection(direction(goal.frame_id, 1.0));
      place->insert(std::move(retreat));
      task.add(std::move(place));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("return to work pose",
                                                         pipeline);
      stage->setGroup(kArmGroup);
      stage->setGoal(kWorkPose);
      task.add(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(
          "close gripper at work pose", joint_interpolation);
      stage->setGroup(kGripperGroup);
      stage->setGoal(kClosedPose);
      task.add(std::move(stage));
    }
    return task;
  }

  void execute(const std::shared_ptr<GoalHandle> &handle) {
    const auto goal = handle->get_goal();
    auto result = std::make_shared<PickPlace::Result>();
    const std::vector<std::string> temporary_ids{goal->object_id};
    try {
      feedback(handle, "planning", "build planning scene");
      apply_scene(*goal);
      auto task = create_task(*goal);
      feedback(handle, "planning", "search complete task solutions");
      const auto plan_result = task.plan();
      result->solution_count = static_cast<std::uint32_t>(task.numSolutions());
      if (!plan_result || task.solutions().empty()) {
        std::ostringstream explanation;
        task.explainFailure(explanation);
        const auto candidate_failures = failure_summary(task, "candidate IK");
        result->error_code = plan_result.val;
        result->message = explanation.str();
        if (!candidate_failures.empty()) {
          result->message += "Candidate summary: " + candidate_failures;
        }
        cleanup_scene(temporary_ids);
        handle->abort(result);
        return;
      }
      const auto best = std::min_element(
          task.solutions().begin(), task.solutions().end(),
          [](const auto &left, const auto &right) {
            return grasp_solution_rank(*left) < grasp_solution_rank(*right);
          });
      const auto *solution = best->get();
      for (const auto &complete : task.solutions())
        RCLCPP_INFO(get_logger(), "complete candidate %zu, insertion +%.3f mm, motion cost %.6f%s",
                    grasp_candidate_index(*complete), grasp_depth(*complete) * 1000.0, complete->cost(),
                    complete.get() == solution ? " (selected)" : "");
      result->selected_cost = solution->cost();
      feedback(handle, "planned", "selected complete candidate " +
                   std::to_string(grasp_candidate_index(*solution)) + " / 加深 " +
                   std::to_string(grasp_depth(*solution) * 1000.0) + " mm",
               task.numSolutions(), solution->cost());
      task.introspection().publishSolution(*solution);
      feedback(handle, "executing", "execute selected task solution",
               task.numSolutions(), solution->cost());
      const auto execute_result = task.execute(*solution);
      result->error_code = execute_result.val;
      result->message =
          execute_result ? "pick and place complete" : "task execution failed";
      cleanup_scene(temporary_ids);
      if (execute_result) {
        handle->succeed(result);
      } else {
        handle->abort(result);
      }
    } catch (const std::exception &error) {
      result->error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      result->message = error.what();
      cleanup_scene(temporary_ids);
      handle->abort(result);
    }
  }

  moveit::planning_interface::PlanningSceneInterface scene_;
  double octomap_resolution_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_subscription_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> sensor_tf_;
  std::mutex cloud_mutex_;
  std::condition_variable cloud_ready_;
  builtin_interfaces::msg::Time filtered_stamp_;
  rclcpp_action::Server<PickPlace>::SharedPtr server_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
