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

#include "trunk_perception/algo/track/matcher/distance_measurement/distance_base.h"
#include "trunk_perception/algo/track/matcher/graph/gated_hungarian_bigraph_matcher.h"
#include "trunk_perception/algo/track/matcher/matcher_base.h"

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
   * @param objects_tracked tracking objects
   * @param objects_detected detection objects
   * @param assignments pair assigned tracking objects and detection objects
   * @param unassigned_tracks unassigned tracking objects
   * @param unassigned_objects unassigned detection objects
   * @return int
   */
  int Match(const std::vector<Tracklet>& objects_tracked, const std::vector<Object>& objects_detected,
            std::vector<TrackObjectPair>* assignments, std::vector<size_t>* unassigned_tracks,
            std::vector<size_t>* unassigned_objects) override;

 private:
  /**
   * @brief compute objects tracked and objects detected cost matrix
   *
   * @param objects_tracked current tracking objects
   * @param objects_detected current detection objects
   */
  void computeCostMatrix(const std::vector<Tracklet>& objects_tracked, const std::vector<Object>& objects_detected);

  /**
   * @brief solve assign result tracking objects and detection objects
   *
   * @param assignments pair assigned tracking objects and detection objects
   * @param unassigned_tracks unassigned tracking objects
   * @param unassigned_objects unassigned detection objects
   */
  void solve(std::vector<TrackObjectPair>* assignments, std::vector<size_t>* unassigned_tracks,
             std::vector<size_t>* unassigned_objects);

 private:
  std::unique_ptr<DistanceBase> distance_measurement_ptr_ = nullptr;
  std::unique_ptr<GatedHungarianMatcher<float>> hungarian_matcher_ptr_ = nullptr;
  MatcherOptions matcher_options_;
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END