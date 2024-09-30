/**
 * @file object_distance_measurement.cpp
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "trunk_perception/algo/track/matcher/distance_measurement/object_distance_measurement.h"
#include "trunk_perception/algo/track/common/geometric_algo.h"
#include "trunk_perception/algo/track/matcher/distance_measurement/general_distance.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

int ObjectDistanceMeasurement::Init(const YAML::Node& config) {
  try {
    params_.method = config["method"].as<std::string>();
    if (params_.method == "NORMAL_DISTANCE") {
      params_.max_distance = config["max_distance"].as<float>();
      params_.max_velocity = config["max_velocity"].as<float>();
    } else if (params_.method == "FUSION_DISTANCE") {
      params_.weight = config["weight"].as<std::vector<float>>();
    } else {
      TERROR << "[ObjectDistanceMeasurement] compute distance method error!";
    }
  } catch (const std::exception& e) {
    TFATAL << "[ObjectDistanceMeasurement] LoadYAMLConfig failed! " << e.what();
    return 1;
  }

  return 0;
}

float ObjectDistanceMeasurement::ComputeDistance(const Tracklet& track, const Object& object) {
  float distance = 0.0F;
  if (params_.method == "NORMAL_DISTANCE") {
    distance = normalDistance(track, object);
  } else if (params_.method == "FUSION_DISTANCE") {
    distance = fusionDistance(track, object);
  } else {
    TERROR << "[ObjectDistanceMeasurement] compute distance method error!";
  }

  return distance;
}

float ObjectDistanceMeasurement::fusionDistance(const Tracklet& track, const Object& object) {
  float distance = 0.0F;
  constexpr float delta = 1E-10F;
  const double time_diff = object.timestamp - track.current_tracking_object.timestamp;

  if (params_.weight[0] > delta) {
    const float dis0 = params_.weight[0] * LocationDistance(track, object, time_diff);
    // TERROR << "dis0: " << dis0;
    distance += dis0;
  }

  if (params_.weight[1] > delta) {
    const float dis1 = params_.weight[1] * DirectionDistance(track, object, time_diff);
    // TERROR << "dis1: " << dis1;
    distance += dis1;
  }

  if (params_.weight[2] > delta) {
    const float dis2 = params_.weight[2] * BboxSizeDistance(track, object, time_diff);
    // TERROR << "dis2: " << dis2;
    distance += dis2;
  }

  if (params_.weight[3] > delta) {
    const float dis3 = params_.weight[3] * PointNumDistance(track, object, time_diff);
    // TERROR << "dis3: " << dis3;
    distance += dis3;
  }

  if (params_.weight[4] > delta) {
    const float dis4 = params_.weight[4] * CentroidShiftDistance(track, object, time_diff);
    // TERROR << "dis4: " << dis4;
    distance += dis4;
  }

  if (params_.weight[5] > delta) {
    const float dis5 = params_.weight[5] * BboxIouDistance(track, object, time_diff);
    // TERROR << "dis5: " << dis5;
    distance += dis5;
  }

  if (params_.weight[6] > delta) {
    const float dis6 = params_.weight[6] * BboxTailDistance(track, object, time_diff);
    // TERROR << "dis6: " << dis6;
    distance += dis6;
  }

  return distance;
}

float ObjectDistanceMeasurement::normalDistance(const Tracklet& track, const Object& object) {
  const double dt = object.timestamp - track.current_tracking_object.timestamp;

  if (dt < 0.0) return 1.0F;

  // 计算IOU
  const auto& bbox_detected = object.bbox;
  const auto& bbox_tracked = track.current_tracking_object.bbox;
  const double iou = getOverlapRate(bbox_tracked.corners2d, bbox_detected.corners2d);

  // 计算距离
  const float distance_error = (bbox_detected.center - bbox_tracked.center).head(2).norm();

  // 距离判断
  if (iou < 0.01 && distance_error > params_.max_distance) return 1.0F;

  // 计算速度距离
  const double velo = distance_error / dt;
  const float distance = std::max(0.0, std::min(1.0, velo / params_.max_velocity));
  return distance;
}

TRUNK_PERCEPTION_LIB_NAMESPACE_END