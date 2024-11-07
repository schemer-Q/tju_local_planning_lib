/**
 * @file tail_center_distance.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-11-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "trunk_perception/algo/track/matcher/distance_measurement/tail_center_distance.h"
#include "trunk_perception/algo/track/common/geometric_algo.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

int TailCenterDistance::Init(const YAML::Node& config) {
  try {
    params_.max_distance = config["max_distance"].as<float>();
    params_.max_velocity = config["max_velocity"].as<float>();
    params_.max_theta_diff = config["max_theta_diff"].as<float>();
    params_.max_projection_distance = config["max_projection_distance"].as<float>();
  } catch (const std::exception& e) {
    TFATAL << "[TailCenterDistance] init failed! " << e.what();
    return 1;
  }

  return 0;
}

float TailCenterDistance::ComputeDistance(const Tracklet& track, const Object& object) {
  const double dt = object.timestamp - track.current_tracking_object.timestamp;
  if (dt < 0.0) return 1.0F;

  // 计算IOU
  const auto& bbox_detected = object.bbox;
  const auto& bbox_tracked = track.current_tracking_object.bbox;
  const double iou = getOverlapRate(bbox_tracked.corners2d, bbox_detected.corners2d);

  // 计算距离
  const auto& tail_point_detected = object.tail_center_feature.tail_center_point;
  const auto& tail_point_tracked = track.current_tracking_object.tail_center_feature.tail_center_point;
  const Eigen::Vector2f& tail_point_vector = (tail_point_detected - tail_point_tracked).cast<float>();
  const Eigen::Vector2f& center_vector = (bbox_detected.center - bbox_tracked.center).head(2);
  const float tail_point_vector_norm = tail_point_vector.norm();
  const float center_vector_norm = center_vector.norm();

  float distance_error = 0.0F;
  float projection_dist = 0.0F;
  const Eigen::Vector2f direction = object.bbox.direction.head(2);
  if (tail_point_vector_norm < center_vector_norm) {
    distance_error = tail_point_vector_norm;
    const Eigen::Vector2f projection = (tail_point_vector.dot(direction) / direction.dot(direction)) * direction;
    projection_dist = (tail_point_vector - projection).norm();
  } else {
    distance_error = center_vector_norm;
    const Eigen::Vector2f projection = (center_vector.dot(direction) / direction.dot(direction)) * direction;
    projection_dist = (center_vector - projection).norm();
  }

  // 距离判断
  if (iou < 0.01 && distance_error > params_.max_distance) return 1.0F;

  // 航向角判断
  const float theta_diff = getAngleDiff(bbox_tracked.theta, bbox_detected.theta);
  if (std::abs(theta_diff) > params_.max_theta_diff) return 1.0F;
  if (std::abs(theta_diff) <= params_.max_theta_diff && projection_dist > params_.max_projection_distance) return 1.0F;

  // 计算速度距离
  const double velo = distance_error / dt;
  const float distance = std::max(0.0, std::min(1.0, velo / params_.max_velocity));
  return distance;
}

REGISTER_DISTANCE("TailCenterDistance", TailCenterDistance)

TRUNK_PERCEPTION_LIB_NAMESPACE_END