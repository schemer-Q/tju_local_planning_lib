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

#include "trunk_perception/algo/track/matcher/one_stage_matcher.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

int OneStageMatcher::Init(const YAML::Node& config) {
  try {
    if (config["MatcherOptions"].IsDefined()) {
      matcher_options_.cost_thresh = config["MatcherOptions"]["cost_thresh"].as<float>();
      matcher_options_.bound_value = config["MatcherOptions"]["bound_value"].as<float>();
    }
  } catch (const std::exception& e) {
    TFATAL << "[OneStageMatcher] LoadYAMLConfig failed! " << e.what();
    return 1;
  }

  // distance measurement instantiation
  distance_measurement_ptr_ = std::make_unique<ObjectDistanceMeasurement>();
  distance_measurement_ptr_->Init(config);

  // hungarian matcher instantiation
  hungarian_matcher_ptr_ = std::make_unique<GatedHungarianMatcher<float>>(1000);

  return 0;
}

int OneStageMatcher::Match(const std::vector<Tracklet>& objects_tracked, const std::vector<Object>& objects_detected,
                           std::vector<TrackObjectPair>* assignments, std::vector<size_t>* unassigned_tracks,
                           std::vector<size_t>* unassigned_objects) {
  // compute cost matrix
  computeCostMatrix(objects_tracked, objects_detected);

  // optimizer solve
  solve(assignments, unassigned_tracks, unassigned_objects);

  return 0;
}

void OneStageMatcher::computeCostMatrix(const std::vector<Tracklet>& objects_tracked,
                                        const std::vector<Object>& objects_detected) {
  if (!hungarian_matcher_ptr_) {
    TFATAL << "[OneStageMatcher.computeCostMatrix] hungarian_matcher_ptr_ is nullptr!";
    return;
  }

  if (!distance_measurement_ptr_) {
    TFATAL << "[OneStageMatcher.computeCostMatrix] distance_measurement_ptr_ is nullptr!";
    return;
  }

  const size_t sz_track = objects_tracked.size();
  const size_t sz_detect = objects_detected.size();
  auto global_costs = hungarian_matcher_ptr_->mutable_global_costs();
  global_costs->Resize(sz_track, sz_detect);
  for (size_t i = 0; i < sz_track; ++i) {
    for (size_t j = 0; j < sz_detect; ++j) {
      (*global_costs)(i, j) = distance_measurement_ptr_->ComputeDistance(objects_tracked[i], objects_detected[j]);
    }
  }
}

void OneStageMatcher::solve(std::vector<TrackObjectPair>* assignments, std::vector<size_t>* unassigned_tracks,
                            std::vector<size_t>* unassigned_objects) {
  if (!hungarian_matcher_ptr_) {
    TFATAL << "[OneStageMatcher.solve] hungarian_matcher_ptr_ is nullptr!";
    return;
  }

  const auto opt_flag = GatedHungarianMatcher<float>::OptimizeFlag::OPTMIN;
  hungarian_matcher_ptr_->Match(matcher_options_.cost_thresh, matcher_options_.bound_value, opt_flag, assignments,
                                unassigned_tracks, unassigned_objects);
}

REGISTER_MATCHER("OneStageMatcher", OneStageMatcher)

TRUNK_PERCEPTION_LIB_NAMESPACE_END