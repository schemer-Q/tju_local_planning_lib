/**
 * @file tracker_objects_match.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 跟踪器与检测结果的关联
 * @version 0.1
 * @date 2024-10-30
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <utility>
#include <vector>

#include "trunk_perception/algo/track/matcher/graph/gated_hungarian_bigraph_matcher.h"
#include "trunk_perception/app/target_fusion/data_associate/track_object_distance.h"
#include "trunk_perception/app/target_fusion/data_fusion/tracker.h"
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

typedef std::pair<size_t, size_t> TrackMeasurmentPair;

struct AssociationResult {
  std::vector<TrackMeasurmentPair> track_measurment_pairs;
  std::vector<size_t> unassigned_track_indices;
  std::vector<size_t> unassigned_measurment_indices;
};

class TrackerObjectsMatch {
 public:
  TrackerObjectsMatch();
  ~TrackerObjectsMatch() = default;

  std::uint32_t Init(const YAML::Node& config);

  /**
   * @brief 激光雷达数据关联
   *
   * @param trackers 跟踪器列表
   * @param lidar_objects 激光雷达检测到的目标
   * @param result 关联结果
   */
  void Match(const std::vector<TrackerPtr>& trackers, const std::vector<LidarMeasureFrame::Ptr>& lidar_objects,
             AssociationResult& association_result);

  /**
   * @brief 毫米波雷达数据关联
   *
   * @param trackers 跟踪器列表
   * @param front_radar_objects 毫米波雷达检测到的目标
   * @param association_result 关联结果
   */
  void Match(const std::vector<TrackerPtr>& trackers,
             const std::vector<ars430::RadarMeasureFrame::Ptr>& front_radar_objects,
             AssociationResult& association_result);

 private:
  std::shared_ptr<TrackObjectDistance> lidar_distance_compute_ptr_ = nullptr;
  std::shared_ptr<TrackObjectDistance> front_radar_distance_compute_ptr_ = nullptr;
  std::shared_ptr<GatedHungarianMatcher<float>> hungarian_matcher_ptr_ = nullptr;

  float filter_thresh_front_radar_velocity_abs_ = 5.0;
  float filter_thresh_front_radar_velocity_rel_ = 30.0;

  float distance_weight_front_radar_velocity_ = 0.7;
  float distance_weight_front_radar_position_ = 0.3;

  float hungarian_match_cost_thresh_lidar_ = 0.99;
  float hungarian_match_bound_value_lidar_ = 1.00;

  float hungarian_match_cost_thresh_radar_ = 4.99;
  float hungarian_match_bound_value_radar_ = 5.00;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END