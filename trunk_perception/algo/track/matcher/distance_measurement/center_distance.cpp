/**
 * @file center_distance.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-11-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "trunk_perception/algo/track/matcher/distance_measurement/center_distance.h"
#include "trunk_perception/algo/track/common/geometric_algo.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

int CenterDistance::Init(const YAML::Node& config) {
  try {
    params_.max_distance = config["max_distance"].as<float>();
    params_.max_velocity = config["max_velocity"].as<float>();
  } catch (const std::exception& e) {
    TFATAL << "[CenterDistance] init failed! " << e.what();
    return 1;
  }

  return 0;
}

float CenterDistance::ComputeDistance(const Tracklet& track, const Object& object) {
  const double dt = object.timestamp - track.current_tracking_object.timestamp;
  if (dt < 0.0) return 1.0F;

  // 计算IOU
  const auto& bbox_detected = object.bbox;
  const auto& bbox_tracked = track.current_tracking_object.bbox;
  const double iou = getOverlapRate(bbox_tracked.corners2d, bbox_detected.corners2d);

  // IOU和距离判断
  const float distance_error = getDistanceError(bbox_tracked, bbox_detected);
  if (iou < 0.01 && distance_error > params_.max_distance) return 1.0F;

  // 计算速度
  const double velocity = (bbox_detected.center.head(2) - bbox_tracked.center.head(2)).norm() / dt;
  const float distance = std::max(0.0, std::min(1.0, velocity / params_.max_velocity));
  return distance;
}

float CenterDistance::getDistanceError(const BoundingBox& track_bbox, const BoundingBox& detect_bbox) {
  float min_distance = std::min(1000.0F, (track_bbox.center.head(2) - detect_bbox.center.head(2)).norm());
  for (int i = -1; i <= 1; ++i) {
    for (int n = 0; n < 4; ++n) {
      const int j = (i + n + 4) % 4;
      const Eigen::Vector2f corner_distance = track_bbox.corners2d.col(j) - detect_bbox.corners2d.col(n);
      min_distance = std::min(min_distance, corner_distance.norm());
    }
  }
  return min_distance;
}

REGISTER_DISTANCE("CenterDistance", CenterDistance)

TRUNK_PERCEPTION_LIB_NAMESPACE_END