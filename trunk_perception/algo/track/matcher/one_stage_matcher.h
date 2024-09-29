/**
 * @file one_stage_matcher.h
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "distance_measurement/object_distance_measurement.h"
#include "hungarian/hungarian_algo.h"
#include "matcher_base.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

class OneStageMatcher : virtual public MatcherBase {
 public:
  OneStageMatcher() = default;
  ~OneStageMatcher() override = default;

  /**
   * @brief matcher init
   *
   * @param config yaml node
   * @return int
   */
  int Init(const YAML::Node& config) override;

  /**
   * @brief match objects detected and objects tracked
   *
   * @param objects_detected detection objects
   * @param objects_tracked tracking objects
   * @param assignment match result
   * @return int
   */
  int Match(const std::vector<Tracklet>& objects_tracked, const std::vector<Object>& objects_detected,
            std::vector<int>& assignment) override;

 private:
  /**
   * @brief compute objects tracked and objects detected Cost Matrix
   *
   * @param objects_tracked current tracking objects
   * @param objects_detected current detection objects
   * @param cost cost matrix
   */
  void computeCostMatrix(const std::vector<Tracklet>& objects_tracked, const std::vector<Object>& objects_detected,
                         std::vector<std::vector<double>>& cost);

  /**
   * @brief
   *
   * @param cost
   * @param assignment
   */
  void solve(const std::vector<std::vector<double>>& cost, std::vector<int>& assignment);

 private:
  std::shared_ptr<AssignmentProblemSolver> solver_ptr_ = nullptr;                  // optimizer solver
  std::shared_ptr<ObjectDistanceMeasurement> distance_measurement_ptr_ = nullptr;  // object distance measurement
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END