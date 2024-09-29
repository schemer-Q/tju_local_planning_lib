/**
 * @file one_stage_matcher.cpp
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "one_stage_matcher.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

int OneStageMatcher::Init(const YAML::Node& config) {
  solver_ptr_ = std::make_shared<AssignmentProblemSolver>();
  distance_measurement_ptr_ = std::make_shared<ObjectDistanceMeasurement>();
  distance_measurement_ptr_->Init(config);
  return 0;
}

int OneStageMatcher::Match(const std::vector<Tracklet>& objects_tracked, const std::vector<Object>& objects_detected,
                           std::vector<int>& assignment) {
  // compute cost matrix
  std::vector<std::vector<double>> costs;
  computeCostMatrix(objects_tracked, objects_detected, costs);

  // optimizer solve
  solve(costs, assignment);

  return 0;
}

void OneStageMatcher::computeCostMatrix(const std::vector<Tracklet>& objects_tracked,
                                        const std::vector<Object>& objects_detected,
                                        std::vector<std::vector<double>>& costs) {
  const size_t sz_track = objects_tracked.size();
  const size_t sz_detect = objects_detected.size();
  costs = std::vector<std::vector<double>>(sz_track, std::vector<double>(sz_detect, 1.0));

  for (size_t i = 0; i < sz_track; ++i) {
    for (size_t j = 0; j < sz_detect; ++j) {
      costs[i][j] = distance_measurement_ptr_->ComputeDistance(objects_tracked[i], objects_detected[j]);
    }
  }
}

void OneStageMatcher::solve(const std::vector<std::vector<double>>& costs, std::vector<int>& assignment) {
  assignment.clear();
  if (costs.size() > 0) {
    solver_ptr_->Solve(costs, assignment, AssignmentProblemSolver::optimal);

    for (size_t i = 0; i < assignment.size(); ++i) {
      if (assignment[i] == -1 || costs[i][assignment[i]] >= 0.99) {
        assignment[i] = -1;
      }
    }
  }
}

TRUNK_PERCEPTION_LIB_NAMESPACE_END