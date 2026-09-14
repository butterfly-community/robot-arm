#pragma once
#include "mtc_context.hpp"
#include "collision_workspace.hpp"
#include "fingertips.hpp"
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/generate_pose.h>

namespace stararm_mtc {
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
inline geometry_msgs::msg::Pose grasp_origin(const PickPlace::Goal &goal, std::size_t index,
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
inline std::vector<DepthPose> grasp_depth_poses(const PickPlace::Goal &goal, double resolution,
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
inline std::vector<DepthPose> allowed_grasp_depth_poses(
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

inline double collision_workspace_radius(const moveit::core::RobotModel& model,
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

// ComputeIK replaces SubTrajectory; identity lives on forwarded properties.
// Read end properties first, then wrappers/children with the original reducers.
template <class T, class Merge>
T solution_property(const mtc::SolutionBase &solution, const char *name, T empty, Merge merge) {
  if (solution.end() && solution.end()->properties().hasProperty(name))
    return solution.end()->properties().get<T>(name);
  if (const auto *wrapped = dynamic_cast<const mtc::WrappedSolution *>(&solution))
    return solution_property(*wrapped->wrapped(), name, empty, merge);
  T value = empty;
  if (const auto *sequence = dynamic_cast<const mtc::SolutionSequence *>(&solution))
    for (const auto *child : sequence->solutions())
      value = merge(value, solution_property(*child, name, empty, merge));
  return value;
}

inline std::size_t grasp_candidate_index(const mtc::SolutionBase &solution) {
  return solution_property(solution, "grasp_candidate_index", std::numeric_limits<std::size_t>::max(),
      [](auto a, auto b) { return std::min(a, b); });
}
inline double grasp_depth(const mtc::SolutionBase &solution) {
  return solution_property(solution, "grasp_depth_m", 0.0,
      [](auto a, auto b) { return std::max(a, b); });
}
inline bool grasp_planar_centered(const mtc::SolutionBase &solution) {
  return solution_property(solution, "grasp_planar_centered", false,
      [](auto a, auto b) { return a || b; });
}

// Dimensionless mean fraction of the arm's existing joint travel. Native MTC
// computes joint distances along every waypoint (including reversals), not
// merely endpoint distance. No gripper travel or hard-coded joint IDs/weights.
inline mtc::cost::PathLength arm_motion_cost(const moveit::core::RobotModel &model) {
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
inline double grasp_center_distance(const PickPlace::Goal &goal, std::size_t index, double depth_m,
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
inline double grasp_engagement_distance(const PickPlace::Goal &goal, std::size_t index, double depth_m,
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
inline double grasp_remaining_standoff(const PickPlace::Goal &goal, std::size_t index, double depth_m,
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
inline double grasp_geometry_cost(const PickPlace::Goal &goal, std::size_t index, double depth_m,
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
inline double grasp_closing_span(const PickPlace::Goal &goal, std::size_t index) {
  const auto object_rotation = pose_transform(goal.object_pose).linear().eval();
  const auto closing_axis = pose_transform(goal.grasp_poses.at(index)).linear().col(0).eval();
  const Eigen::Vector3d size(goal.object_size.x, goal.object_size.y, goal.object_size.z);
  return (object_rotation.transpose() * closing_axis).cwiseAbs().dot(size);
}

// Transform TWO CAD tip points, not the whole fingers or a proxy TCP axis.
// Among the candidates admitted by the user-selected tip-angle filter, use
// height difference as a tie-break after quality. Never edit the candidate pose.
inline double grasp_fingertip_level_cost(const PickPlace::Goal &goal, std::size_t index,
                                  const stararm::FingertipPair &tcp_tips) {
  const auto pose = pose_transform(goal.grasp_poses.at(index));
  return stararm::fingertip_level_cost({pose * tcp_tips[0], pose * tcp_tips[1]});
}

inline double grasp_quality_cost(const PickPlace::Goal &goal, std::size_t index, double depth_m,
                          bool planar_centered = false) {
  const double diagonal = Eigen::Vector3d(goal.object_size.x, goal.object_size.y,
                                         goal.object_size.z).norm();
  return 1.0 - goal.grasp_confidences.at(index) +
    (grasp_closing_span(goal, index) +
     grasp_engagement_distance(goal, index, depth_m, planar_centered)) / diagonal;
}

// Evaluate geometry once per proposal, not at every O(N log N) comparison.
// Stable sorting preserves the previous order for exactly equal costs.
inline void sort_grasp_poses(std::vector<DepthPose> &poses, const PickPlace::Goal &goal) {
  std::vector<std::pair<double, DepthPose>> scored;
  scored.reserve(poses.size());
  for (auto &pose : poses) {
    const double cost = grasp_quality_cost(goal, pose.candidate_index, pose.depth_m, pose.planar_centered);
    scored.emplace_back(cost, std::move(pose));
  }
  std::stable_sort(scored.begin(), scored.end(), [](const auto &a, const auto &b) {
    return a.first < b.first;
  });
  for (std::size_t i = 0; i < poses.size(); ++i) poses[i] = std::move(scored[i].second);
}

// Use the same grasp objective while SEARCHING as when ranking complete paths.
// Native defaults add distance-to-default-joints in ComputeIK and path length
// in motion stages. With a bounded complete pool that selects short/easy paths
// before our final grasp comparator ever sees the other candidates.
// MTC's public cost-term interface changes search preference only: failure
// status, IK, trajectories, collision checking and execution stay native.
inline void configure_grasp_search_cost(mtc::Task &task, const PickPlace::Goal &goal) {
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
inline std::vector<RankedGrasp> rank_complete_grasps(
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


} // namespace stararm_mtc
