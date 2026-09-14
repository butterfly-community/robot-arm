#include "task_factory.hpp"
#include "task_scene.hpp"
#include <moveit/task_constructor/stages/fixed_state.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <iomanip>
#include <thread>

using namespace stararm_mtc;
using GoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;

namespace {
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
    scene_ = std::make_unique<TaskScene>(*this, octomap_resolution_);
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
      auto task = create_task(*goal, *resources_, octomap_resolution_);
      scene_->apply(*goal, task.getRobotModel());
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
        scene_->cleanup(temporary_ids);
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
        auto refinement = create_task(*goal, *resources_, octomap_resolution_, seed.candidate_index);
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
      scene_->cleanup(temporary_ids);
      if (execute_result) {
        handle->succeed(result);
      } else {
        handle->abort(result);
      }
    } catch (const std::exception &error) {
      result->error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      result->message = error.what();
      scene_->cleanup(temporary_ids);
      handle->abort(result);
    }
  }

  std::unique_ptr<TaskScene> scene_;
  std::shared_ptr<PlanningResources> resources_;
  std::mutex planning_mutex_;
  double octomap_resolution_;
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
