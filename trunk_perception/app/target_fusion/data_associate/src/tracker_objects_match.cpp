#include "trunk_perception/app/target_fusion/data_associate/tracker_objects_match.h"
#include <cstddef>
#include <numeric>

#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

TrackerObjectsMatch::TrackerObjectsMatch() {
  hungarian_matcher_ptr_ = std::make_shared<GatedHungarianMatcher<float>>(1000);
  front_radar_distance_compute_ptr_ = std::make_shared<TrackObjectDistance>();
  lidar_distance_compute_ptr_ = std::make_shared<TrackObjectDistance>();
	front_vision_distance_compute_ptr_ = std::make_shared<TrackObjectDistance>();
}

std::uint32_t TrackerObjectsMatch::Init(const YAML::Node& config) {
  YAML::Node radar_dis_config, lidar_dis_config, front_vision_dis_config;
  try {
    radar_dis_config = config["DistanceCompute"]["FrontRadar"];
    lidar_dis_config = config["DistanceCompute"]["Lidar"];
		front_vision_dis_config = config["DistanceCompute"]["FrontVision"];
    hungarian_match_cost_thresh_lidar_ = config["HungarianMatcher"]["Lidar"]["MatchCostThresh"].as<float>();
    hungarian_match_bound_value_lidar_ = config["HungarianMatcher"]["Lidar"]["MatchBoundValue"].as<float>();
    hungarian_match_cost_thresh_radar_ = config["HungarianMatcher"]["FrontRadar"]["MatchCostThresh"].as<float>();
    hungarian_match_bound_value_radar_ = config["HungarianMatcher"]["FrontRadar"]["MatchBoundValue"].as<float>();
    hungarian_match_cost_thresh_front_vision_ = config["HungarianMatcher"]["FrontVision"]["MatchCostThresh"].as<float>();
    hungarian_match_bound_value_front_vision_ = config["HungarianMatcher"]["FrontVision"]["MatchBoundValue"].as<float>();
  } catch (const std::exception& e) {
    TFATAL << "TrackerObjectsMatch Init failed: " << e.what();
    return ErrorCode::YAML_CONFIG_ERROR;
  }

  uint32_t ret = ErrorCode::SUCCESS;

  ret = front_radar_distance_compute_ptr_->Init(radar_dis_config);
  if (ret != ErrorCode::SUCCESS) {
    return ret;
  }

  ret = lidar_distance_compute_ptr_->Init(lidar_dis_config);
  if (ret != ErrorCode::SUCCESS) {
    return ret;
  }

	ret = front_vision_distance_compute_ptr_->Init(front_vision_dis_config);
  if (ret != ErrorCode::SUCCESS) {
    return ret;
  }

  return ErrorCode::SUCCESS;
}

void TrackerObjectsMatch::Match(const std::vector<TrackerPtr>& trackers,
                                const std::vector<LidarMeasureFrame::Ptr>& lidar_objects,
                                AssociationResult& association_result) {
  association_result = AssociationResult();

  if (trackers.empty() || lidar_objects.empty()) {
    association_result.unassigned_track_indices = std::vector<size_t>(trackers.size());
    association_result.unassigned_measurment_indices = std::vector<size_t>(lidar_objects.size());
    std::iota(association_result.unassigned_track_indices.begin(), association_result.unassigned_track_indices.end(),
              0);
    std::iota(association_result.unassigned_measurment_indices.begin(),
              association_result.unassigned_measurment_indices.end(), 0);

    return;
  }

  // 计算距离矩阵
  const size_t sz_track = trackers.size();
  const size_t sz_detect = lidar_objects.size();
  auto global_costs = hungarian_matcher_ptr_->mutable_global_costs();
  global_costs->Resize(sz_track, sz_detect);

  for (size_t i = 0; i < sz_track; ++i) {
    for (size_t j = 0; j < sz_detect; ++j) {
      (*global_costs)(i, j) = lidar_distance_compute_ptr_->Compute(trackers[i], lidar_objects[j]);
    }
  }

  // 匈牙利匹配
  const auto opt_flag = GatedHungarianMatcher<float>::OptimizeFlag::OPTMIN;
  hungarian_matcher_ptr_->Match(hungarian_match_cost_thresh_lidar_, hungarian_match_bound_value_lidar_, opt_flag,
                                &association_result.track_measurment_pairs,
                                &association_result.unassigned_track_indices,
                                &association_result.unassigned_measurment_indices);
}

void TrackerObjectsMatch::Match(const std::vector<TrackerPtr>& trackers,
                                const std::vector<ars430::RadarMeasureFrame::Ptr>& front_radar_objects,
                                AssociationResult& association_result) {
  association_result = AssociationResult();

  if (trackers.empty() || front_radar_objects.empty()) {
    association_result.unassigned_track_indices = std::vector<size_t>(trackers.size());
    association_result.unassigned_measurment_indices = std::vector<size_t>(front_radar_objects.size());
    std::iota(association_result.unassigned_track_indices.begin(), association_result.unassigned_track_indices.end(),
              0);
    std::iota(association_result.unassigned_measurment_indices.begin(),
              association_result.unassigned_measurment_indices.end(), 0);
    return;
  }

  // 计算距离矩阵
  const size_t sz_track = trackers.size();
  const size_t sz_detect = front_radar_objects.size();
  auto global_costs = hungarian_matcher_ptr_->mutable_global_costs();
  global_costs->Resize(sz_track, sz_detect);

  for (size_t i = 0; i < sz_track; ++i) {
    for (size_t j = 0; j < sz_detect; ++j) {
      (*global_costs)(i, j) = front_radar_distance_compute_ptr_->Compute(trackers[i], front_radar_objects[j]);
    }
  }

  // 匈牙利匹配
  const auto opt_flag = GatedHungarianMatcher<float>::OptimizeFlag::OPTMIN;
  hungarian_matcher_ptr_->Match(hungarian_match_cost_thresh_radar_, hungarian_match_bound_value_radar_, opt_flag,
                                &association_result.track_measurment_pairs,
                                &association_result.unassigned_track_indices,
                                &association_result.unassigned_measurment_indices);
}

// 前向视觉目标与航迹进行匹配 @author zzg 2024-12-13 
void TrackerObjectsMatch::Match(const std::vector<TrackerPtr>& trackers,
																const std::vector<VisionMeasureFrame::Ptr>& front_vision_objects,
																AssociationResult& association_result) {
	association_result = AssociationResult();
	if (trackers.empty() || front_vision_objects.empty()) {
		association_result.unassigned_track_indices = std::vector<size_t>(trackers.size());
		association_result.unassigned_measurment_indices = std::vector<size_t>(front_vision_objects.size());
		std::iota(association_result.unassigned_track_indices.begin(), association_result.unassigned_track_indices.end(),
							0);
		std::iota(association_result.unassigned_measurment_indices.begin(), association_result.unassigned_measurment_indices.end(),
							0);
		return;
	}

	// 计算距离矩阵
	const size_t sz_track = trackers.size();
	const size_t sz_detect = front_vision_objects.size();
	auto global_costs = hungarian_matcher_ptr_->mutable_global_costs();
	global_costs->Resize(sz_track, sz_detect);

	for (size_t i = 0; i < sz_track; ++i) {
		for (size_t j = 0; j < sz_detect; ++j) {
			(*global_costs)(i, j) = front_vision_distance_compute_ptr_->Compute(trackers[i], front_vision_objects[j]);
		}
	}

	// 匈牙利匹配
	const auto opt_flag = GatedHungarianMatcher<float>::OptimizeFlag::OPTMIN;
	hungarian_matcher_ptr_->Match(hungarian_match_cost_thresh_front_vision_, hungarian_match_bound_value_front_vision_, opt_flag,
																&association_result.track_measurment_pairs,
																&association_result.unassigned_track_indices,
																&association_result.unassigned_measurment_indices);
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END