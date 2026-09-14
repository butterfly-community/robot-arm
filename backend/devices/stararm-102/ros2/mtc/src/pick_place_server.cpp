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
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/cost_terms.h>
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
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "stararm_102_mtc/action/pick_place.hpp"
#include "collision_workspace.hpp"
#include "release_ik.hpp"
#include "fingertips.hpp"

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
constexpr double kDefaultVelocityScaling = 0.125; // User halved grasp/place speed; physical limits unchanged.
constexpr double kClosingVelocityScaling = kDefaultVelocityScaling / 2.0;

Eigen::Isometry3d pose_transform(const geometry_msgs::msg::Pose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                                     pose.orientation.y, pose.orientation.z).toRotationMatrix();
  return result;
}

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

// Preserve measured initial support contact while departing with an attached
// object. This is the same conditional ACM formerly used for Cartesian lift.
planning_scene::PlanningScenePtr support_departure_scene(
    const planning_scene::PlanningSceneConstPtr &from) {
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
    return planning_scene;
}

// A named arm target avoids requiring a continuous fixed-orientation vertical
// IK path, without leaving the departure endpoint's wrist orientation arbitrary.
// The gripper and attached object are preserved; only the arm group moves.
class SupportAwarePipelinePlanner final : public mtc::solvers::PipelinePlanner {
public:
  explicit SupportAwarePipelinePlanner(const rclcpp::Node::SharedPtr &node) : PipelinePlanner(node) {
    setMaxVelocityScalingFactor(kDefaultVelocityScaling);
  }
  using PipelinePlanner::plan;
  Result plan(const planning_scene::PlanningSceneConstPtr &from,
              const planning_scene::PlanningSceneConstPtr &to,
              const moveit::core::JointModelGroup *group, double timeout,
              robot_trajectory::RobotTrajectoryPtr &result,
              const moveit_msgs::msg::Constraints &constraints) override {
    auto prepared = support_departure_scene(from);
    auto target = to->diff();
    target->getAllowedCollisionMatrixNonConst() = prepared->getAllowedCollisionMatrix();
    return PipelinePlanner::plan(prepared, target, group, timeout, result, constraints);
  }
};

// Resolve free-orientation release endpoints near the incoming held posture,
// then let the unchanged MoveIt pipeline plan to collision-free joint goals.
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
    const auto started = std::chrono::steady_clock::now();
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
    const auto states = release_ik_states(from->getCurrentState(), group, link, offset.translation(),
        Eigen::Vector3d(point.point.x, point.point.y, point.point.z), timeout);
    Result status{false, "No collision-free release IK/path near the incoming held posture"};
    for (std::size_t i = 0; i < states.size(); ++i) {
      // This is native scene/constraint checking, including the attached object;
      // proximity ranks alternatives, it never excuses a collision.
      if (!from->isStateValid(states[i], goal, group->getName())) continue;
      const double remaining = timeout - std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count();
      if (remaining <= 0.0) break;
      auto joint_goal = kinematic_constraints::constructGoalConstraints(
          states[i], group, properties().get<double>("goal_joint_tolerance"));
      // Retain the original object-centre region as well: joint-goal tolerance
      // must not silently enlarge the requested position/height tolerance.
      joint_goal.position_constraints = goal.position_constraints;
      status = PipelinePlanner::plan(from, group, joint_goal, remaining / (states.size() - i),
                                     result, path_constraints);
      if (status) return status;
    }
    return status;
  }
};

struct DepthPose {
  geometry_msgs::msg::PoseStamped pose;
  std::size_t candidate_index;
  double depth_m;
  bool planar_centered = false;
};

// Preserve the model orientation and height. The second translation proposal
// aligns with the observed centre in the SAME ground plane used by this task.
// Slanted insertion alone couples forward engagement to moving below ground.
// Both proposals still require the complete native IK/collision/transport path;
// this is not a target correction, new force threshold or model rescore.
geometry_msgs::msg::Pose grasp_origin(const PickPlace::Goal &goal, std::size_t index,
                                     bool planar_centered) {
  auto pose = goal.grasp_poses.at(index);
  if (planar_centered) {
    pose.position.x = goal.object_pose.position.x;
    pose.position.y = goal.object_pose.position.y;
  }
  return pose;
}

// Search insertion from both original and planar-centred origins, without an
// orientation restriction. Bound insertion by the observed object's far face
// projected onto TCP +Z. Use the existing scene resolution, including both
// endpoints; this is a discrete search, not a claim of an exact global optimum.
std::vector<DepthPose> grasp_depth_poses(const PickPlace::Goal &goal, double resolution,
                                      std::optional<std::size_t> selected = std::nullopt,
                                      bool all_planar_origins = false) {
  std::vector<DepthPose> result;
  const auto &object = goal.object_pose;
  const Eigen::Vector3d center(object.position.x, object.position.y, object.position.z);
  const Eigen::Quaterniond object_rotation(object.orientation.w, object.orientation.x,
                                          object.orientation.y, object.orientation.z);
  const Eigen::Vector3d size(goal.object_size.x, goal.object_size.y, goal.object_size.z);
  for (std::size_t index = 0; index < goal.grasp_poses.size(); ++index) {
    if (selected && index != *selected)
      continue;
    for (const bool planar_centered : {false, true}) {
      // Coarse complete-plan search stays on original model poses. The existing
      // finite single-candidate refinement explores both origins afterwards.
      // all_planar_origins only computes a conservative collision-map extent.
      if (planar_centered && !selected && !all_planar_origins) continue;
      const auto pose = grasp_origin(goal, index, planar_centered);
      const auto &original = goal.grasp_poses[index];
      if (planar_centered && pose.position.x == original.position.x &&
          pose.position.y == original.position.y)
        continue; // Identical proposal, not another search path.
      const Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x,
                                        pose.orientation.y, pose.orientation.z);
      const Eigen::Vector3d axis = rotation * Eigen::Vector3d::UnitZ();
      const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
      const Eigen::Vector3d object_axis = object_rotation.conjugate() * axis;
      const double far_face = axis.dot(center - position) + 0.5 * object_axis.cwiseAbs().dot(size);
      const double maximum = std::max(0.0, far_face);
      const std::size_t steps = static_cast<std::size_t>(std::ceil(maximum / resolution));
      for (std::size_t remaining = steps + 1; remaining > 0; --remaining) {
        const auto step = remaining - 1; // Deepest first; still retain the complete grid.
        const double depth = std::min(step * resolution, maximum);
        DepthPose variant;
        variant.pose.header.frame_id = goal.frame_id;
        variant.pose.pose = pose;
        variant.pose.pose.position.x += axis.x() * depth;
        variant.pose.pose.position.y += axis.y() * depth;
        variant.pose.pose.position.z += axis.z() * depth;
        variant.candidate_index = index;
        variant.depth_m = depth;
        variant.planar_centered = planar_centered;
        result.push_back(std::move(variant));
      }
    }
  }
  return result;
}

// Use the same open CAD tip points in coarse and depth-refinement searches.
// Keep original candidate identities, scores and poses; filter before ComputeIK.
std::vector<DepthPose> allowed_grasp_depth_poses(
    const PickPlace::Goal &goal, double resolution,
    const stararm::FingertipPair &tcp_tips,
    std::optional<std::size_t> selected = std::nullopt) {
  auto poses = grasp_depth_poses(goal, resolution, selected);
  poses.erase(std::remove_if(poses.begin(), poses.end(), [&](const DepthPose &variant) {
    const auto transform = pose_transform(variant.pose.pose);
    return !stararm::fingertip_tilt_allowed(
        {transform * tcp_tips[0], transform * tcp_tips[1]});
  }), poses.end());
  return poses;
}

double collision_workspace_radius(const moveit::core::RobotModel& model,
                                  const PickPlace::Goal& goal, double resolution) {
  const Eigen::Vector3d center(goal.object_pose.position.x, goal.object_pose.position.y,
                               goal.object_pose.position.z);
  const double half_diagonal = 0.5 * Eigen::Vector3d(
      goal.object_size.x, goal.object_size.y, goal.object_size.z).norm();
  double attached_extent = 0.0;
  for (const auto& variant : grasp_depth_poses(goal, resolution, std::nullopt, true)) {
    const auto& p = variant.pose.pose.position;
    attached_extent = std::max(attached_extent,
        (Eigen::Vector3d(p.x, p.y, p.z) - center).norm() + half_diagonal);
  }
  // Include held geometry at ANY generated insertion and an entire voxel
  // diagonal at the boundary. This is a conservative map crop, not an IK limit.
  return collision_reach(model, model.getLinkModel(kTcpFrame), attached_extent)
      + std::sqrt(3.0) * resolution;
}

class GeneratePoses final : public mtc::stages::GeneratePose {
public:
  GeneratePoses(const std::string &name,
                std::vector<DepthPose> poses)
      : mtc::stages::GeneratePose(name),
        poses_(std::move(poses)) {}

  void reset() override {
    active_ = nullptr;
    active_scene_.reset();
    next_ = 0;
    mtc::stages::GeneratePose::reset();
  }

  bool canCompute() const override {
    return !poses_.empty() && (active_ || mtc::stages::GeneratePose::canCompute());
  }

  void compute() override {
    if (!canCompute()) return;
    if (!active_) {
      active_ = upstream_solutions_.pop();
      active_scene_ = active_->end()->scene()->diff();
      next_ = 0;
    }
    const auto &upstream = *active_;
    {
      mtc::InterfaceState state(active_scene_);
      forwardProperties(*upstream.end(), state);
      const auto &variant = poses_[next_++];
      state.properties().set("target_pose", variant.pose);
      state.properties().set("grasp_candidate_index", variant.candidate_index);
      state.properties().set("grasp_depth_m", variant.depth_m);
      state.properties().set("grasp_planar_centered", variant.planar_centered);
      mtc::SubTrajectory trajectory;
      trajectory.setComment("candidate " + std::to_string(variant.candidate_index) +
                            " depth +" + std::to_string(variant.depth_m * 1000.0) + " mm" +
                            (variant.planar_centered ? " planar-centred" : " original"));
      spawn(std::move(state), std::move(trajectory));
    }
    if (next_ == poses_.size()) {
      active_ = nullptr;
      active_scene_.reset();
    }
  }

private:
  std::vector<DepthPose> poses_;
  const mtc::SolutionBase *active_ = nullptr;
  planning_scene::PlanningScenePtr active_scene_;
  std::size_t next_ = 0;
};

// Preserve candidate identity through MTC's replacement/wrapper stages.
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

bool grasp_planar_centered(const mtc::SolutionBase &solution) {
  if (solution.end() && solution.end()->properties().hasProperty("grasp_planar_centered"))
    return solution.end()->properties().get<bool>("grasp_planar_centered");
  if (const auto *wrapped = dynamic_cast<const mtc::WrappedSolution *>(&solution))
    return grasp_planar_centered(*wrapped->wrapped());
  if (const auto *sequence = dynamic_cast<const mtc::SolutionSequence *>(&solution))
    for (const auto *child : sequence->solutions())
      if (grasp_planar_centered(*child)) return true;
  return false;
}

// Dimensionless mean fraction of the arm's existing joint travel. Native MTC
// computes joint distances along every waypoint (including reversals), not
// merely endpoint distance. No gripper travel or hard-coded joint IDs/weights.
mtc::cost::PathLength arm_motion_cost(const moveit::core::RobotModel &model) {
  const auto &joints = model.getJointModelGroup(kArmGroup)->getActiveJointModels();
  std::map<std::string, double> weights;
  for (const auto *joint : joints)
    weights.emplace(joint->getName(), 1.0 / (joints.size() * joint->getMaximumExtent()));
  return mtc::cost::PathLength(std::move(weights));
}

struct RankedGrasp {
  const mtc::SolutionBase *solution;
  std::size_t candidate_index;
  double depth_m;
  double confidence;
  double motion_cost;
  double closing_span_m;
  double normalized_span;
  double center_distance_m;
  double normalized_center_distance;
  double engagement_distance_m;
  double normalized_engagement_distance;
  double remaining_standoff_m;
  double geometry_cost;
  double fingertip_level_cost;
  bool planar_centered = false;

  double cost() const {
    return (1.0 - confidence) + normalized_span + normalized_engagement_distance;
  }

  auto rank_key() const {
    return std::make_tuple(cost(), fingertip_level_cost, motion_cost, closing_span_m,
                           -confidence, candidate_index, planar_centered);
  }
};

// Compare the FINAL insertion-adjusted TCP with the observed object centre.
// Path length only measures arm motion; it does not measure grasp engagement.
// This is a soft geometric preference, not a contact/COM estimate or rejection.
double grasp_center_distance(const PickPlace::Goal &goal, std::size_t index, double depth_m,
                             bool planar_centered = false) {
  const auto tcp = pose_transform(grasp_origin(goal, index, planar_centered));
  const auto center = pose_transform(goal.object_pose).translation().eval();
  return (tcp.translation() + tcp.linear().col(2) * depth_m - center).norm();
}

// This arm's TCP is the closed fingertip, not the centre of the finger pads.
// The object should enter behind that tip along TCP -Z. Euclidean distance to
// the tip incorrectly penalized useful insertion as much as stopping short:
// real run 009 ranked a shallow miss above a narrower .965-score complete grasp.
// Distance to the inward approach ray preserves transverse centering and
// penalizes remaining stand-off, without penalizing already achieved depth.
// This is a soft rank, NOT a contact assertion, rejection or pose modification.
double grasp_engagement_distance(const PickPlace::Goal &goal, std::size_t index, double depth_m,
                                 bool planar_centered = false) {
  const auto tcp = pose_transform(grasp_origin(goal, index, planar_centered));
  const auto center = pose_transform(goal.object_pose).translation().eval();
  Eigen::Vector3d relative = tcp.linear().transpose() * (center - tcp.translation());
  relative.z() = std::max(0.0, relative.z() - depth_m);
  return relative.norm();
}

// A transverse miss and a tip still in front of the object are different:
// recorded successful runs 021-023 had 8-17 mm transverse error but the centre
// was already behind the closed tip. Empty runs 024-025 had 12-14 mm of actual
// stand-off. Keep this directional diagnostic separate from transverse error.
// It is not a gate or a forced world direction.
double grasp_remaining_standoff(const PickPlace::Goal &goal, std::size_t index, double depth_m,
                                bool planar_centered = false) {
  const auto tcp = pose_transform(grasp_origin(goal, index, planar_centered));
  const auto center = pose_transform(goal.object_pose).translation().eval();
  return std::max(0.0, tcp.linear().col(2).dot(center - tcp.translation()) - depth_m);
}

// Scale displacement by the observed envelope projected onto each TCP axis.
// A centimetre of stand-off on a thin object is not the same as a centimetre
// across a broad face. Unlike absolute axial priority, this also penalizes
// moving completely off the object's side. No tuned weights, class-specific
// dimensions, rejection threshold, contact assertion or world-up constraint.
double grasp_geometry_cost(const PickPlace::Goal &goal, std::size_t index, double depth_m,
                           bool planar_centered = false) {
  const auto tcp = pose_transform(grasp_origin(goal, index, planar_centered));
  const auto object = pose_transform(goal.object_pose);
  Eigen::Vector3d local = tcp.linear().transpose() * (object.translation() - tcp.translation());
  local.z() = std::max(0.0, local.z() - depth_m);
  const Eigen::Vector3d size(goal.object_size.x, goal.object_size.y, goal.object_size.z);
  const Eigen::Vector3d extents = (tcp.linear().transpose() * object.linear()).cwiseAbs() * size;
  return local.cwiseQuotient(extents).norm();
}

// Width of the observed envelope along the fingers' closing direction. The
// StarArm descriptor declares canonical X, and its canonical-base -> TCP
// rotation is identity (only translation). Thus returned TCP X is closing X.
// This is a projection, NOT solid-object contact or force-closure evidence.
// No object class, world-axis preference, nominal size or acceptance threshold.
double grasp_closing_span(const PickPlace::Goal &goal, std::size_t index) {
  const auto object_rotation = pose_transform(goal.object_pose).linear().eval();
  const auto closing_axis = pose_transform(goal.grasp_poses.at(index)).linear().col(0).eval();
  const Eigen::Vector3d size(goal.object_size.x, goal.object_size.y, goal.object_size.z);
  return (object_rotation.transpose() * closing_axis).cwiseAbs().dot(size);
}

// Transform TWO CAD tip points, not the whole fingers or a proxy TCP axis.
// Among the candidates admitted by the user-selected tip-angle filter, use
// height difference as a tie-break after quality. Never edit the candidate pose.
double grasp_fingertip_level_cost(const PickPlace::Goal &goal, std::size_t index,
                                  const stararm::FingertipPair &tcp_tips) {
  const auto pose = pose_transform(goal.grasp_poses.at(index));
  return stararm::fingertip_level_cost({pose * tcp_tips[0], pose * tcp_tips[1]});
}

double grasp_quality_cost(const PickPlace::Goal &goal, std::size_t index, double depth_m,
                          bool planar_centered = false) {
  const double diagonal = Eigen::Vector3d(goal.object_size.x, goal.object_size.y,
                                         goal.object_size.z).norm();
  return 1.0 - goal.grasp_confidences.at(index) +
    (grasp_closing_span(goal, index) +
     grasp_engagement_distance(goal, index, depth_m, planar_centered)) / diagonal;
}

// Use the same grasp objective while SEARCHING as when ranking complete paths.
// Native defaults add distance-to-default-joints in ComputeIK and path length
// in motion stages. With a bounded complete pool that selects short/easy paths
// before our final grasp comparator ever sees the other candidates.
// MTC's public cost-term interface changes search preference only: failure
// status, IK, trajectories, collision checking and execution stay native.
void configure_grasp_search_cost(mtc::Task &task, const PickPlace::Goal &goal) {
  PickPlace::Goal geometry;
  geometry.grasp_poses = goal.grasp_poses;
  geometry.grasp_confidences = goal.grasp_confidences;
  geometry.object_pose = goal.object_pose;
  geometry.object_size = goal.object_size;
  const auto insertion = std::make_shared<mtc::LambdaCostTerm>(
      [geometry = std::move(geometry)](const mtc::SubTrajectory &solution) {
        const auto index = grasp_candidate_index(solution);
        const auto depth = grasp_depth(solution);
        return grasp_quality_cost(geometry, index, depth, grasp_planar_centered(solution));
      });
  const auto zero = std::make_shared<mtc::cost::Constant>(0.0);
  task.stages()->traverseRecursively([&](const mtc::Stage &stage, unsigned int) {
    // Traversal exposes a const stage even while constructing a mutable task.
    auto &configured = const_cast<mtc::Stage &>(stage);
    if (dynamic_cast<const GeneratePoses *>(&stage) ||
        dynamic_cast<const mtc::stages::ComputeIK *>(&stage))
      configured.setCostTerm(insertion);
    else if (!dynamic_cast<const mtc::ContainerBase *>(&stage))
      configured.setCostTerm(zero);
    // Containers retain the native sum. Arm travel remains the final secondary
    // comparator, computed from actual trajectories, not lost or hard-limited.
    return true;
  });
}

// First keep each candidate's DEEPEST complete solution, even if a shallower
// variant is cheaper. Closing span is a dimensionless SOFT term, normalized by
// the observed diagonal, alongside original model quality.
// Fingertip engagement distance uses the same diagonal, without a tuned weight.
// Geometry remains a SOFT part of grasp quality, not an absolute priority.
// Recorded 033 picked a .488-score / 102 mm span grasp solely for a smaller
// centering proxy, despite a complete .906-score / 59 mm span alternative.
// Rank combined quality before arm travel; retain geometry as a diagnostic.
// Recorded real run 020 chose a
// lower-model-quality, wider and shallower miss just to save joint travel.
// All entries already have complete collision-checked paths; a short path is
// a secondary preference, not compensation for poorer grasp engagement.
// Absolute span priority selected a shallow .602-score grasp over a .806-score
// deeper complete plan for only 3 mm less span in a real trial. No tuned weights,
// acceptance thresholds or object classes. CAD fingertip level only breaks
// equal-quality ties, not a forced top-down approach. Extra
// insertion is NOT comparable across different original poses.
std::vector<RankedGrasp> rank_complete_grasps(
    const std::vector<const mtc::SolutionBase *> &solutions,
    const PickPlace::Goal &goal, const mtc::cost::PathLength &motion_cost,
    const stararm::FingertipPair &tcp_tips) {
  std::map<std::pair<std::size_t, bool>, RankedGrasp> deepest;
  const double diagonal = Eigen::Vector3d(goal.object_size.x, goal.object_size.y,
                                          goal.object_size.z).norm();
  for (const auto *solution : solutions) {
    const auto index = grasp_candidate_index(*solution);
    std::string comment;
    const auto span = grasp_closing_span(goal, index);
    const auto depth = grasp_depth(*solution);
    const auto centered = grasp_planar_centered(*solution);
    const auto key = std::make_pair(index, centered);
    const auto distance = grasp_center_distance(goal, index, depth, centered);
    const auto engagement = grasp_engagement_distance(goal, index, depth, centered);
    RankedGrasp candidate{solution, index, depth, goal.grasp_confidences.at(index),
                          solution->computeCost(motion_cost, comment), span, span / diagonal,
                          distance, distance / diagonal, engagement, engagement / diagonal,
                          grasp_remaining_standoff(goal, index, depth, centered),
                          grasp_geometry_cost(goal, index, depth, centered),
                          grasp_fingertip_level_cost(goal, index, tcp_tips), centered};
    auto previous = deepest.find(key);
    if (previous == deepest.end() ||
        std::make_pair(-candidate.depth_m, candidate.motion_cost) <
            std::make_pair(-previous->second.depth_m, previous->second.motion_cost))
      deepest.insert_or_assign(key, candidate);
  }
  std::vector<RankedGrasp> ranked;
  for (const auto &[index, candidate] : deepest)
    ranked.push_back(candidate);
  std::sort(ranked.begin(), ranked.end(), [](const auto &left, const auto &right) {
    return left.rank_key() < right.rank_key();
  });
  return ranked;
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

#include "planning_resources.hpp"

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

  void initialize_planning() {
    resources_ = std::make_shared<PlanningResources>(shared_from_this());
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

  void apply_scene(const PickPlace::Goal &goal, const moveit::core::RobotModelConstPtr& model) {
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
    RCLCPP_INFO(get_logger(), "collision workspace radius %.6f m: %u -> %u points",
                radius, goal.scene_cloud.width * goal.scene_cloud.height, cloud.width * cloud.height);
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
    const auto prepared = std::chrono::steady_clock::now();
    cloud_publisher_->publish(cloud);
    std::unique_lock<std::mutex> lock(cloud_mutex_);
    // Only the corresponding callback certifies that Octomap finished updating.
    // Do not mistake publishing the message for a completed scene update.
    cloud_ready_.wait(lock, [&] { return filtered_stamp_ == cloud.header.stamp; });
    lock.unlock();
    if (!scene_.applyCollisionObject(actual_target))
      throw std::runtime_error("MoveIt rejected restoration of actual target dimensions");
    RCLCPP_INFO(get_logger(), "request %s scene: prepare %.6f s, update/sync %.6f s", goal.request_id.c_str(),
        std::chrono::duration<double>(prepared-begun).count(),
        std::chrono::duration<double>(std::chrono::steady_clock::now()-prepared).count());
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

  mtc::Task create_task(const PickPlace::Goal &goal,
                        std::optional<std::size_t> candidate = std::nullopt) {
    mtc::Task task("", false); // Publish only the final task; no throw-away DDS introspection.
    task.setName("pick and place " + goal.request_id);
    task.setRobotModel(resources_->model);
    task.setProperty("group", kArmGroup);
    task.setProperty("eef", kEndEffector);
    task.setProperty("hand", kGripperGroup);
    task.setProperty("ik_frame", kTcpFrame);

    auto pipeline = resources_->pipeline;
    auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    auto joint_interpolation =
        std::make_shared<CheckedJointInterpolation>();
    pipeline->setMaxVelocityScalingFactor(kDefaultVelocityScaling);
    cartesian->setMaxVelocityScalingFactor(kDefaultVelocityScaling);
    joint_interpolation->setMaxVelocityScalingFactor(kDefaultVelocityScaling);
    auto closing = std::make_shared<CheckedJointInterpolation>();
    closing->setMaxVelocityScalingFactor(kClosingVelocityScaling);

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
      // MTC propagates BACKWARD from the grasp: this finds the pregrasp, not
      // an insertion-depth limit. Let native collision/IK stop the retreat.
      // A finite ray spanning the entire robot envelope cannot truncate a
      // reachable straight segment (triangle inequality); no object-size cap.
      approach->setMinMaxDistance(0.0, 2.0 * collision_reach(*task.getRobotModel()));
      approach->setDirection(direction(kTcpFrame, 1.0));
      pick->insert(std::move(approach));

      auto poses = allowed_grasp_depth_poses(goal, octomap_resolution_,
          stararm::open_fingertips_tcp(task.getRobotModel()), candidate);
      // Emit lazily in the SAME search-cost order. Eagerly inserting thousands
      // of states into MTC's ordered linked lists costs quadratic queue work
      // before the first IK query. All poses remain available after failures;
      // only a complete task solution can terminate the search.
      std::stable_sort(poses.begin(), poses.end(), [&goal](const auto &a, const auto &b) {
        return grasp_quality_cost(goal, a.candidate_index, a.depth_m, a.planar_centered) <
               grasp_quality_cost(goal, b.candidate_index, b.depth_m, b.planar_centered);
      });
      if (poses.empty()) {
        std::ostringstream message;
        message << "没有两指尖连线相对任务地平面倾角小于 "
                << stararm::kMaximumFingertipTiltDegrees << "° 的抓取候选";
        throw std::runtime_error(message.str());
      }
      auto generator = std::make_unique<GeneratePoses>("grasp candidates", std::move(poses));
      generator->properties().configureInitFrom(mtc::Stage::PARENT);
      generator->setMonitoredStage(open_stage);
      auto grasp_ik = std::make_unique<mtc::stages::ComputeIK>(
          "candidate IK", std::move(generator));
      grasp_ik->setGroup(kArmGroup);
      grasp_ik->setEndEffector(kEndEffector);
      // Native timeout still governs search. Do not stop at an application-
      // selected count; zero is NOT unlimited in MTC (it runs no IK attempts).
      grasp_ik->setMaxIKSolutions(std::numeric_limits<uint32_t>::max());
      grasp_ik->setForwardedProperties({"grasp_candidate_index", "grasp_depth_m", "grasp_planar_centered"});
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
                                                         closing);
      close->setGroup(kGripperGroup);
      close->setGoal(kClosedPose);
      // As in the official MTC pick pipeline, allow finger/target contact and
      // validate the closing path with the planner. A segmented OBB cannot
      // reject rim grasps by pretending that an open object is a solid cuboid.
      pick->insert(std::move(close));

      auto attach =
          std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      attach->attachObject(goal.object_id, kTcpFrame);
      pick->insert(std::move(attach));

      auto depart = std::make_unique<mtc::stages::MoveTo>(
          "carry to work pose", resources_->carry);
      depart->setGroup(kArmGroup);
      depart->setGoal(kWorkPose);
      pick->insert(std::move(depart));

      auto restore_support = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "restore support collision after departure");
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
          "transport to release point", resources_->release);
      release->setGroup(kArmGroup);
      // Move the attached object's centre to the requested placement point.
      // MTC resolves its candidate-specific offset from the attachment state;
      // targeting TCP instead caused the observed far-edge placement drift.
      release->setIKFrame(goal.object_id);
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

      // Return directly with the arm planner after release. A separate vertical
      // Cartesian retreat unnecessarily required continuous fixed-orientation IK.
      // The detached object and all collision checks remain in the scene.
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
    configure_grasp_search_cost(task, goal);
    return task;
  }

  void execute(const std::shared_ptr<GoalHandle> &handle) {
    const auto goal = handle->get_goal();
    auto result = std::make_shared<PickPlace::Result>();
    const std::vector<std::string> temporary_ids{goal->object_id};
    // Include cleanup on both success and exceptions in the same serialization.
    std::unique_lock<std::mutex> planning_lock(planning_mutex_);
    try {
      if (goal->grasp_poses.size() != goal->grasp_confidences.size() ||
          !std::all_of(goal->grasp_confidences.begin(), goal->grasp_confidences.end(),
                       [](double confidence) { return std::isfinite(confidence); }))
        throw std::runtime_error("Grasp poses require matching finite model confidences");
      const auto planning_started = std::chrono::steady_clock::now();
      feedback(handle, "planning", "build planning scene");
      auto task = create_task(*goal);
      apply_scene(*goal, task.getRobotModel());
      const auto scene_ready = std::chrono::steady_clock::now();
      // Publish the selected solution and search statistics once below, rather
      // than streaming every iteration. Action feedback still reports progress.
      task.enableIntrospection(false);
      feedback(handle, "planning", "search complete task solutions");
      // Search is already ordered by the grasp objective. Stop at the first
      // COMPLETE collision-checked pick/place path, not the first valid IK.
      // Failed candidates still lead to the next pose; selected-pose depth
      // refinement below remains exhaustive. No deadline discards an unsolved task.
      const auto plan_result = task.plan(1);
      const auto coarse_ready = std::chrono::steady_clock::now();
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
      std::vector<const mtc::SolutionBase *> complete_solutions;
      for (const auto &complete : task.solutions())
        complete_solutions.push_back(complete.get());
      auto ranked = rank_complete_grasps(complete_solutions, *goal,
                                        arm_motion_cost(*task.getRobotModel()),
                                        stararm::open_fingertips_tcp(task.getRobotModel()));
      { // Refine the selected grasp before the single full execution.
        // A bounded complete pool does NOT exhaust a candidate's insertion grid.
        // Real replay: candidate 418 had a valid +25 mm complete plan, while
        // the bounded search returned only +5 mm. Refine the selected candidate
        // using the SAME factory, collision scene and full pick/place stages.
        // Same observation and start state; no robot command or model call during this search.
        const auto seed = ranked.front();
        feedback(handle, "planning", "核对所选候选的最大可行夹取深度");
        auto refinement = create_task(*goal, seed.candidate_index);
        moveit_msgs::msg::PlanningScene frozen;
        seed.solution->start()->scene()->getPlanningSceneMsg(frozen);
        auto frozen_scene = std::make_shared<planning_scene::PlanningScene>(refinement.getRobotModel());
        frozen_scene->setPlanningSceneMsg(frozen);
        refinement.stages()->remove(0);
        refinement.stages()->insert(std::make_unique<mtc::stages::FixedState>(
            "current state", frozen_scene), 0);
        refinement.enableIntrospection(false);
        refinement.plan(0); // Exhaust this finite one-candidate grid, not all candidates.
        std::vector<const mtc::SolutionBase *> refined_solutions;
        for (const auto &complete : refinement.solutions())
          refined_solutions.push_back(complete.get());
        auto refined = rank_complete_grasps(refined_solutions, *goal,
                                           arm_motion_cost(*refinement.getRobotModel()),
                                           stararm::open_fingertips_tcp(refinement.getRobotModel()));
        if (!refined.empty() &&
            ((refined.front().planar_centered == seed.planar_centered &&
              refined.front().depth_m > seed.depth_m) ||
             refined.front().rank_key() < seed.rank_key())) {
          RCLCPP_INFO(get_logger(), "candidate %zu complete-pose refinement: %.3f -> %.3f mm",
                      seed.candidate_index, seed.depth_m * 1000, refined.front().depth_m * 1000);
          task = std::move(refinement);
          ranked = std::move(refined);
        }
        // No improvement keeps the already validated complete seed; it does
        // not execute a failed IK or substitute an unchecked translated pose.
        result->solution_count = static_cast<std::uint32_t>(task.numSolutions());
      }
      const auto &best = ranked.front();
      const auto plan_ready = std::chrono::steady_clock::now();
      RCLCPP_INFO(get_logger(), "request %s planning: scene %.6f s, coarse %.6f s, refine %.6f s, total %.6f s",
          goal->request_id.c_str(), std::chrono::duration<double>(scene_ready-planning_started).count(),
          std::chrono::duration<double>(coarse_ready-scene_ready).count(),
          std::chrono::duration<double>(plan_ready-coarse_ready).count(),
          std::chrono::duration<double>(plan_ready-planning_started).count());
      const auto *solution = best.solution;
      for (const auto &candidate : ranked)
        RCLCPP_INFO(get_logger(),
                    "deepest complete candidate %zu, insertion +%.3f mm, model %.6f, "
                    "normalized arm travel %.6f, total cost %.6f, closing span %.3f mm, normalized span %.6f, "
                    "TCP center distance %.3f mm, normalized center distance %.6f, engagement distance %.3f mm, remaining standoff %.3f mm, geometry cost %.6f, fingertip level cost %.6f, planar centered %s%s",
                    candidate.candidate_index, candidate.depth_m * 1000.0, candidate.confidence,
                    candidate.motion_cost, candidate.cost(), candidate.closing_span_m * 1000.0, candidate.normalized_span,
                    candidate.center_distance_m * 1000.0, candidate.normalized_center_distance,
                    candidate.engagement_distance_m * 1000.0,
                    candidate.remaining_standoff_m * 1000.0,
                    candidate.geometry_cost,
                    candidate.fingertip_level_cost,
                    candidate.planar_centered ? "true" : "false",
                    candidate.solution == solution ? " (selected)" : "");
      result->selected_cost = best.cost();
      std::ostringstream selection;
      selection << std::fixed << std::setprecision(3)
                << "候选 " << best.candidate_index << " / 加深 " << best.depth_m * 1000.0
                << " mm / 平面居中 " << (best.planar_centered ? "是" : "否")
                << " / 夹持跨度 " << best.closing_span_m * 1000.0
                << " mm / TCP距中心 " << best.center_distance_m * 1000.0
                << " mm / 夹取偏差 " << best.engagement_distance_m * 1000.0
                << " mm / 剩余接近 " << best.remaining_standoff_m * 1000.0
                << " mm / 张开指尖连线离水平 "
                << std::asin(std::clamp(best.fingertip_level_cost, 0.0, 1.0)) * 180.0 / M_PI
                << "° / 模型分 " << best.confidence << " / 关节行程代价 " << best.motion_cost;
      feedback(handle, "planned", selection.str(), task.numSolutions(), best.cost());
      task.introspection().publishTaskDescription();
      task.introspection().publishTaskState();
      task.introspection().publishSolution(*solution);
      feedback(handle, "executing", "执行：" + selection.str(),
               task.numSolutions(), best.cost());
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
  std::shared_ptr<PlanningResources> resources_;
  std::mutex planning_mutex_;
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
  node->initialize_planning();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
