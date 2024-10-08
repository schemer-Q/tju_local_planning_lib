/**
 * @file general_distance.cpp
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-10
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "trunk_perception/algo/track/matcher/distance_measurement/general_distance.h"
#include "trunk_perception/algo/track/common/geometric_algo.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

float LocationDistance(const Tracklet& track, const Object& object, const double time_diff) {
  const auto& track_center = track.current_tracking_object.bbox.center;
  const auto& track_velcity = track.current_tracking_object.velocity;
  const Eigen::Vector3f predict_center = track_center + track_velcity * time_diff;
  const Eigen::Vector3f measure_predict_diff = object.bbox.center - predict_center;

  float location_dist = measure_predict_diff.head(2).norm();
  Eigen::Vector2f ref_dir = track.current_tracking_object.velocity.head(2);
  const float speed = ref_dir.norm();
  ref_dir /= speed;

  if (speed > 2) {
    Eigen::Vector2f ref_o_dir = Eigen::Vector2f(ref_dir(1), -ref_dir(0));
    const float dx = ref_dir(0) * measure_predict_diff(0) + ref_dir(1) * measure_predict_diff(1);
    const float dy = ref_o_dir(0) * measure_predict_diff(0) + ref_o_dir(1) * measure_predict_diff(1);
    location_dist = std::sqrt(dx * dx * 0.5 + dy * dy * 2);
  }

  return location_dist;
}

float DirectionDistance(const Tracklet& track, const Object& object, const double time_diff) {
  const auto& track_theta = track.current_tracking_object.bbox.theta;
  const auto& object_theta = object.bbox.theta;
  const float direction_dist = std::abs((object_theta - track_theta) * 0.5F / M_PI);
  return direction_dist;
}

float BboxSizeDistance(const Tracklet& track, const Object& object, const double time_diff) {
  const Eigen::Vector3f& old_bbox_dir = track.current_tracking_object.bbox.direction;
  const Eigen::Vector3f& new_bbox_dir = object.bbox.direction;
  const Eigen::Vector3f& old_bbox_size = track.current_tracking_object.bbox.size;
  const Eigen::Vector3f& new_bbox_size = object.bbox.size;

  float bbox_size_dist = 0.0F;
  double dot_val_00 = std::abs(old_bbox_dir(0) * new_bbox_dir(0) + old_bbox_dir(1) * new_bbox_dir(1));
  double dot_val_01 = std::abs(old_bbox_dir(0) * new_bbox_dir(1) - old_bbox_dir(1) * new_bbox_dir(0));
  float temp_val_0 = 0.0F;
  float temp_val_1 = 0.0F;
  if (dot_val_00 > dot_val_01) {
    temp_val_0 = static_cast<float>(std::abs(old_bbox_size(0) - new_bbox_size(0))) /
                 std::max(old_bbox_size(0), new_bbox_size(0));
    temp_val_1 = static_cast<float>(std::abs(old_bbox_size(1) - new_bbox_size(1))) /
                 std::max(old_bbox_size(1), new_bbox_size(1));
    bbox_size_dist = std::min(temp_val_0, temp_val_1);
  } else {
    temp_val_0 = static_cast<float>(std::abs(old_bbox_size(0) - new_bbox_size(1))) /
                 std::max(old_bbox_size(0), new_bbox_size(1));
    temp_val_1 = static_cast<float>(std::abs(old_bbox_size(1) - new_bbox_size(0))) /
                 std::max(old_bbox_size(1), new_bbox_size(0));
    bbox_size_dist = std::min(temp_val_0, temp_val_1);
  }

  return bbox_size_dist;
}

float PointNumDistance(const Tracklet& track, const Object& object, const double time_diff) {
  if (!track.current_tracking_object.points_ptr || !object.points_ptr) {
    return 0.0F;
  }

  const int track_point_number = static_cast<int>(track.current_tracking_object.points_ptr->size());
  const int object_point_number = static_cast<int>(object.points_ptr->size());
  const int diff_number = std::abs(track_point_number - object_point_number);
  const int max_number = std::max(track_point_number, object_point_number);
  if (max_number == 0UL) return 0.0F;
  const float point_num_dist = static_cast<float>(diff_number) / static_cast<float>(max_number);
  return point_num_dist;
}

float CentroidShiftDistance(const Tracklet& track, const Object& object, const double time_diff) {
  if (!track.current_tracking_object.points_ptr || !object.points_ptr) {
    return 0.0F;
  }

  if (track.current_tracking_object.points_ptr->empty() || object.points_ptr->empty()) {
    return 0.0F;
  }

  const auto measured_points = object.points_ptr->getMatrixXfMap(3, 8, 0);
  const auto predicted_points = track.current_tracking_object.points_ptr->getMatrixXfMap(3, 8, 0);
  const Eigen::Vector3f measured_centroid = measured_points.rowwise().mean();
  Eigen::Vector3f predicted_centroid = predicted_points.rowwise().mean();
  const auto& track_velcity = track.current_tracking_object.velocity;
  predicted_centroid += track_velcity * time_diff;

  float centroid_shift_dist = (measured_centroid - predicted_centroid).head(2).norm();
  return centroid_shift_dist;
}

float BboxIouDistance(const Tracklet& track, const Object& object, const double time_diff) {
  const auto& bbox_tracked = track.current_tracking_object.bbox;
  const auto& bbox_detected = object.bbox;
  const double iou = getOverlapRate(bbox_tracked.corners2d, bbox_detected.corners2d);
  const float bbox_iou_dist = 1 - static_cast<float>(iou);
  return bbox_iou_dist;
}

Eigen::Vector3f getTailMiddlePoint(const common::BoundingBox& bbox) {
  Eigen::Vector3f point = Eigen::Vector3f::Zero();
  std::vector<Eigen::Vector3f> corners;
  for (int i = 0; i < bbox.corners2d.cols(); ++i) {
    corners.emplace_back(Eigen::Vector3f(bbox.corners2d(0, i), bbox.corners2d(1, i), 0.0));
  }
  std::sort(corners.begin(), corners.end(), [&](const auto& a, const auto& b) { return a.x() < b.x(); });
  point = (corners[0] + corners[1]).array() * 0.5F;
  return point;
}

float BboxTailDistance(const Tracklet& track, const Object& object, const double time_diff) {
  Eigen::Vector3f measured_tail = getTailMiddlePoint(object.bbox);
  Eigen::Vector3f predicted_tail = getTailMiddlePoint(track.current_tracking_object.bbox);
  const auto& track_velcity = track.current_tracking_object.velocity;
  predicted_tail += track_velcity * time_diff;

  if (predicted_tail(0) < 15.0f || measured_tail(0) < 15.0f) {
    return 0.0;
  }

  float dist = (measured_tail - predicted_tail).head(2).norm();
  return dist;
}

TRUNK_PERCEPTION_LIB_NAMESPACE_END