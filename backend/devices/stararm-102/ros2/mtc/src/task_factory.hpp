#pragma once
#include "grasp_candidates.hpp"
#include "planning_resources.hpp"
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <sstream>

namespace stararm_mtc {
inline geometry_msgs::msg::Vector3Stamped direction(const std::string &frame_id,
                                             double z) {
  geometry_msgs::msg::Vector3Stamped result;
  result.header.frame_id = frame_id;
  result.vector.z = z;
  return result;
}

inline mtc::Task create_task(const PickPlace::Goal &goal,
    const PlanningResources &resources, double resolution,
    std::optional<std::size_t> candidate = std::nullopt) {
  mtc::Task task("", false); // Publish only the final task; no throw-away DDS introspection.
  task.setName("pick and place " + goal.request_id);
  task.setRobotModel(resources.model);
  task.setProperty("group", kArmGroup);
  task.setProperty("eef", kEndEffector);
  task.setProperty("hand", kGripperGroup);
  task.setProperty("ik_frame", kTcpFrame);

  auto pipeline = resources.pipeline;
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

    auto poses = allowed_grasp_depth_poses(goal, resolution,
        stararm::open_fingertips_tcp(task.getRobotModel()), candidate);
    // Emit lazily in the SAME search-cost order. Eagerly inserting thousands
    // of states into MTC's ordered linked lists costs quadratic queue work
    // before the first IK query. All poses remain available after failures;
    // only a complete task solution can terminate the search.
    sort_grasp_poses(poses, goal);
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
        "carry to work pose", resources.carry);
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
        "transport to release point", resources.release);
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
} // namespace stararm_mtc
