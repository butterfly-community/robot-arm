#pragma once

#include <kdl_parser/kdl_parser.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <trac_ik/trac_ik.hpp>

#include <algorithm>
#include <limits>

// Position-only release IK, seeded by the actual incoming MTC state (normally
// the held work pose). Use the installed TRAC-IK Distance objective, not random
// orientation goals. No wrist orientation or extra joint bounds are imposed.
// The fixed terminal segment matters: the goal is the attached object centre,
// whose offset from the wrist changes with every grasp.
inline std::vector<moveit::core::RobotState> release_ik_states(
    const moveit::core::RobotState& start, const moveit::core::JointModelGroup* group,
    const moveit::core::LinkModel& link, const Eigen::Vector3d& offset,
    const Eigen::Vector3d& target, double timeout) {
  KDL::Tree tree;
  KDL::Chain chain;
  const auto& model = start.getRobotModel();
  const auto base = group->getSolverInstance()->getBaseFrame();
  if (!kdl_parser::treeFromUrdfModel(*model->getURDF(), tree) ||
      !tree.getChain(base, link.getName(), chain))
    throw std::runtime_error("Cannot resolve release IK chain from the current robot model");
  chain.addSegment(KDL::Segment("release_object_centre", KDL::Joint(KDL::Joint::Fixed),
                               KDL::Frame(KDL::Vector(offset.x(), offset.y(), offset.z()))));
  std::vector<std::string> names;
  for (const auto& segment : chain.segments)
    if (segment.getJoint().getType() != KDL::Joint::Fixed)
      names.push_back(segment.getJoint().getName());
  if (names.size() != group->getVariableCount())
    throw std::runtime_error("Release IK chain does not match the arm group");
  KDL::JntArray lower(names.size()), upper(names.size()), seed(names.size()), solution;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto& bounds = model->getVariableBounds(names[i]);
    lower(i) = bounds.min_position_;
    upper(i) = bounds.max_position_;
    seed(i) = start.getVariablePosition(names[i]);
  }
  // Keep the deployed solver's per-query budget and TRAC-IK's documented
  // default numerical epsilon. Neither is a new physical acceptance threshold.
  TRAC_IK::TRAC_IK solver(chain, lower, upper, std::min(timeout, group->getDefaultIKTimeout()),
                          1e-5, TRAC_IK::Distance);
  const Eigen::Vector3d local = start.getGlobalLinkTransform(base).inverse() * target;
  const double unrestricted = std::numeric_limits<float>::max();
  const KDL::Twist tolerance(KDL::Vector::Zero(),
                            KDL::Vector(unrestricted, unrestricted, unrestricted));
  if (solver.CartToJnt(seed, KDL::Frame(KDL::Vector(local.x(), local.y(), local.z())),
                      solution, tolerance) < 0)
    return {};
  std::vector<KDL::JntArray> solutions;
  solver.getSolutions(solutions);
  std::stable_sort(solutions.begin(), solutions.end(), [&](const auto& a, const auto& b) {
    return TRAC_IK::TRAC_IK::JointErr(seed, a) < TRAC_IK::TRAC_IK::JointErr(seed, b);
  });
  std::vector<moveit::core::RobotState> states;
  for (const auto& joints : solutions) {
    auto state = start;
    for (std::size_t i = 0; i < names.size(); ++i)
      state.setVariablePosition(names[i], joints(i));
    state.update();
    states.push_back(std::move(state));
  }
  return states;
}
