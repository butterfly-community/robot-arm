#pragma once
#include <moveit/robot_model_loader/robot_model_loader.hpp>

// Included after the task's planner adapters. Immutable model/plugin resources
// belong to the service; stages, start scenes and solutions belong to each task.
struct PlanningResources {
  robot_model_loader::RobotModelLoader loader;
  moveit::core::RobotModelConstPtr model;
  std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline;
  std::shared_ptr<mtc::solvers::PipelinePlanner> carry;
  std::shared_ptr<mtc::solvers::PipelinePlanner> release;

  explicit PlanningResources(const rclcpp::Node::SharedPtr& node)
      : loader(node), model(loader.getModel()),
        pipeline(std::make_shared<mtc::solvers::PipelinePlanner>(node)),
        carry(std::make_shared<SupportAwarePipelinePlanner>(node)),
        release(std::make_shared<PositionOnlyPlanner>(node)) {
    pipeline->setMaxVelocityScalingFactor(kDefaultVelocityScaling);
    pipeline->init(model);
    carry->init(model);
    release->init(model);
  }
};
