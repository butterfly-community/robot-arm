#pragma once
#include "mtc_context.hpp"
#include "release_ik.hpp"
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

namespace stararm_mtc {
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
inline planning_scene::PlanningScenePtr support_departure_scene(
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


} // namespace stararm_mtc
