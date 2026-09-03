#include <Eigen/Geometry>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
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

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
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
constexpr double kGroundSize = 2.0;
constexpr double kGroundDepth = 1.0;
constexpr std::size_t kPlaceYawSamples = 12;

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

} // namespace

class PickPlaceServer : public rclcpp::Node {
public:
  PickPlaceServer() : Node("stararm_102_mtc") {
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

  std::vector<std::string> apply_scene(const PickPlace::Goal &goal) {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(ground(goal.frame_id));
    objects.push_back(
        box(goal.frame_id, goal.object_id, goal.object_pose, goal.object_size));
    std::vector<std::string> temporary_ids{goal.object_id};
    for (std::size_t index = 0; index < goal.obstacle_ids.size(); ++index) {
      objects.push_back(box(goal.frame_id, goal.obstacle_ids[index],
                            goal.obstacle_poses[index],
                            goal.obstacle_sizes[index]));
      temporary_ids.push_back(goal.obstacle_ids[index]);
    }
    if (!scene_.applyCollisionObjects(objects)) {
      throw std::runtime_error("MoveIt rejected the task planning scene");
    }
    return temporary_ids;
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
        std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto current = std::make_unique<mtc::stages::CurrentState>("current state");
    task.add(std::move(current));

    {
      auto support = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow object support contact");
      support->allowCollisions(goal.object_id, kGroundId, true);
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

    mtc::Stage *pick_stage = nullptr;
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

      auto candidates = std::make_unique<mtc::Alternatives>("compute grasp IK");
      for (std::size_t index = 0; index < goal.grasp_poses.size(); ++index) {
        auto generator = std::make_unique<mtc::stages::GeneratePose>(
            "grasp candidate " + std::to_string(index + 1));
        generator->properties().configureInitFrom(mtc::Stage::PARENT);
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = goal.frame_id;
        target.pose = goal.grasp_poses[index];
        generator->setPose(target);
        generator->setMonitoredStage(open_stage);

        auto grasp_ik = std::make_unique<mtc::stages::ComputeIK>(
            "candidate IK " + std::to_string(index + 1), std::move(generator));
        grasp_ik->setGroup(kArmGroup);
        grasp_ik->setEndEffector(kEndEffector);
        grasp_ik->setIKFrame(Eigen::Isometry3d::Identity(), kTcpFrame);
        grasp_ik->properties().configureInitFrom(mtc::Stage::INTERFACE,
                                                 {"target_pose"});
        candidates->add(std::move(grasp_ik));
      }
      pick->insert(std::move(candidates));

      auto allow = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow gripper object collision");
      allow->allowCollisions(goal.object_id,
                             task.getRobotModel()
                                 ->getJointModelGroup(kGripperGroup)
                                 ->getLinkModelNamesWithCollisionGeometry(),
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
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian);
      lift->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      lift->setIKFrame(kTcpFrame);
      lift->setMinMaxDistance(0.0, goal.object_size.z);
      lift->setDirection(direction(goal.frame_id, 1.0));
      pick->insert(std::move(lift));

      auto restore_support = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "restore support collision after lift");
      restore_support->allowCollisions(goal.object_id, kGroundId, false);
      pick->insert(std::move(restore_support));

      pick_stage = pick.get();
      task.add(std::move(pick));
    }

    {
      auto stage = std::make_unique<mtc::stages::Connect>(
          "transport object",
          mtc::stages::Connect::GroupPlannerVector{{kArmGroup, pipeline}});
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage));
    }

    {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task.properties().exposeTo(place->properties(),
                                 {"eef", "hand", "group", "ik_frame"});
      place->properties().configureInitFrom(
          mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

      auto lower = std::make_unique<mtc::stages::MoveRelative>("lower object",
                                                               cartesian);
      lower->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      lower->setIKFrame(kTcpFrame);
      lower->setMinMaxDistance(0.0, goal.object_size.z);
      lower->setDirection(direction(goal.frame_id, -1.0));
      place->insert(std::move(lower));

      geometry_msgs::msg::PoseStamped attached_object_frame;
      attached_object_frame.header.frame_id = goal.object_id;
      attached_object_frame.pose.orientation.w = 1.0;

      auto place_candidates =
          std::make_unique<mtc::Alternatives>("compute place IK");
      const Eigen::Quaterniond placement_orientation(
          goal.placement_pose.orientation.w, goal.placement_pose.orientation.x,
          goal.placement_pose.orientation.y, goal.placement_pose.orientation.z);
      for (std::size_t index = 0; index < kPlaceYawSamples; ++index) {
        const auto yaw = 2.0 * std::acos(-1.0) *
                         static_cast<double>(index) /
                         static_cast<double>(kPlaceYawSamples);
        const Eigen::Quaterniond orientation =
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            placement_orientation;
        auto generator = std::make_unique<mtc::stages::GeneratePlacePose>(
            "place heading " + std::to_string(index + 1));
        generator->properties().configureInitFrom(mtc::Stage::PARENT,
                                                  {"ik_frame"});
        generator->setObject(goal.object_id);
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = goal.frame_id;
        target.pose = goal.placement_pose;
        target.pose.orientation.x = orientation.x();
        target.pose.orientation.y = orientation.y();
        target.pose.orientation.z = orientation.z();
        target.pose.orientation.w = orientation.w();
        generator->setPose(target);
        generator->setMonitoredStage(pick_stage);
        auto place_ik = std::make_unique<mtc::stages::ComputeIK>(
            "heading IK " + std::to_string(index + 1), std::move(generator));
        place_ik->setGroup(kArmGroup);
        place_ik->setEndEffector(kEndEffector);
        place_ik->setIKFrame(attached_object_frame);
        place_ik->properties().configureInitFrom(mtc::Stage::INTERFACE,
                                                 {"target_pose"});
        place_candidates->add(std::move(place_ik));
      }
      place->insert(std::move(place_candidates));

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
          *task.getRobotModel()->getJointModelGroup(kGripperGroup), false);
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
    std::vector<std::string> temporary_ids;
    try {
      if (goal->obstacle_ids.size() != goal->obstacle_poses.size() ||
          goal->obstacle_ids.size() != goal->obstacle_sizes.size()) {
        throw std::runtime_error("obstacle arrays have different lengths");
      }
      feedback(handle, "planning", "build planning scene");
      temporary_ids = apply_scene(*goal);
      auto task = create_task(*goal);
      feedback(handle, "planning", "search complete task solutions");
      const auto plan_result = task.plan();
      result->solution_count = static_cast<std::uint32_t>(task.numSolutions());
      if (!plan_result || task.solutions().empty()) {
        std::ostringstream explanation;
        task.explainFailure(explanation);
        result->error_code = plan_result.val;
        result->message = explanation.str();
        cleanup_scene(temporary_ids);
        handle->abort(result);
        return;
      }
      const auto &solution = *task.solutions().front();
      result->selected_cost = solution.cost();
      feedback(handle, "planned", "selected complete task solution",
               task.numSolutions(), solution.cost());
      task.introspection().publishSolution(solution);
      feedback(handle, "executing", "execute selected task solution",
               task.numSolutions(), solution.cost());
      const auto execute_result = task.execute(solution);
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
