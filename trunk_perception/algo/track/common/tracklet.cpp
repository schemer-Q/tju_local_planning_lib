/**
 * @file tracklet.cpp
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <cmath>
#include <numeric>

#include "trunk_perception/algo/track/common/geometric_algo.h"
#include "trunk_perception/algo/track/common/tracklet.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

void Tracklet::Predict(const double timestamp) {
  const double dt = timestamp - current_tracking_object.timestamp;
  tracker_method_ptr->Predict(dt, current_tracking_object);

  if (params_.predict_by_velocity && current_tracking_object.lifetime >= params_.trigger_predict_count) {
    const auto& velocity = current_tracking_object.velocity;
    auto& bbox = current_tracking_object.bbox;
    bbox.center += velocity * dt;
    auto& corners2d = bbox.corners2d;
    for (int i = 0; i < corners2d.cols(); ++i) {
      corners2d.col(i) += velocity.head(2) * dt;
    }

    auto& convex_polygon = current_tracking_object.convex_polygon;
    for (int i = 0; i < convex_polygon.cols(); ++i) {
      convex_polygon.col(i) += velocity * dt;
    }
  }

  current_tracking_object.consecutive_lost += 1;
  current_tracking_object.timestamp = timestamp;

  // motion direction update
  history_object_distance_.emplace_back(current_tracking_object.bbox.center.head(2).norm());
  current_tracking_object.motion_direction = checkObjectMotionDirection(history_object_distance_);

  // velocity confidence update
  std::pair<double, Eigen::Vector3f> temp(current_tracking_object.timestamp, current_tracking_object.velocity);
  history_object_velocity_.emplace_front(std::move(temp));
  // current_tracking_object.velocity_confidence = checkVelocityConfidence(history_object_velocity_);
}

void Tracklet::Update(const Object& object) {
  tracker_method_ptr->Update(object, current_tracking_object);

  // bbox confidence update
  const float d_area = object.bbox.size.head(2).prod();
  const float t_area = current_tracking_object.bbox.size.head(2).prod();
  current_tracking_object.bbox_confidence = 1.0F - std::abs(d_area - t_area) / (std::max(d_area, t_area) + 1E-6);

  // bbox theta confidence update
  const auto d_theta = object.bbox.theta;
  const auto t_theta = current_tracking_object.bbox.theta;
  current_tracking_object.theta_confidence = 1.0F - std::abs(getAngleDiff(t_theta, d_theta)) / M_PI;

  current_tracking_object.lifetime += 1;
  current_tracking_object.consecutive_lost = 0;
  current_tracking_object.detect_id = object.detect_id;
  current_tracking_object.timestamp = object.timestamp;
  current_tracking_object.detector_type = object.detector_type;
  current_tracking_object.bbox = object.bbox;
  current_tracking_object.type_probs = object.type_probs;
  current_tracking_object.confidence = object.confidence;
  current_tracking_object.type = object.type;
  current_tracking_object.points_ptr = object.points_ptr;
  current_tracking_object.l_shape_feature = object.l_shape_feature;
  current_tracking_object.tail_center_feature = object.tail_center_feature;
  current_tracking_object.convex_polygon = object.convex_polygon;

  // motion direction update
  history_object_distance_.emplace_back(current_tracking_object.bbox.center.head(2).norm());
  current_tracking_object.motion_direction = checkObjectMotionDirection(history_object_distance_);

  // velocity confidence update
  std::pair<double, Eigen::Vector3f> temp(current_tracking_object.timestamp, current_tracking_object.velocity);
  history_object_velocity_.emplace_front(std::move(temp));
  current_tracking_object.velocity_confidence = checkVelocityConfidence(history_object_velocity_);
}

void Tracklet::TransformToCurrent(const Eigen::Isometry3f& tf) {
  // bbox
  auto& bbox = current_tracking_object.bbox;
  bbox.direction = tf.rotation() * bbox.direction;
  bbox.theta = std::atan2(bbox.direction(1), bbox.direction(0));
  bbox.center = tf * bbox.center;
  auto& corners2d = bbox.corners2d;
  for (int i = 0; i < corners2d.cols(); ++i) {
    const Eigen::Vector3f point(corners2d(0, i), corners2d(1, i), 0.0F);
    corners2d.col(i) = (tf * point).head(2);
  }

  // convex polygon
  auto& convex_polygon = current_tracking_object.convex_polygon;
  for (int i = 0; i < convex_polygon.cols(); ++i) {
    convex_polygon.col(i) = tf * convex_polygon.col(i);
  }

  // velocity
  auto& velocity = current_tracking_object.velocity;
  velocity = tf.rotation() * velocity;

  // acceleration
  auto& acceleration = current_tracking_object.acceleration;
  acceleration = tf.rotation() * acceleration;

  // point cloud
  auto& points_ptr = current_tracking_object.points_ptr;
  if (points_ptr) {
    for (auto& pt : points_ptr->points) {
      pt.getVector3fMap() = tf * pt.getVector3fMap();
    }
  }

  // track point
  auto& track_point = current_tracking_object.track_point;
  track_point = tf * track_point;

  // lshape feature
  {
    auto& center_point = current_tracking_object.l_shape_feature.center_point;
    center_point = (tf.cast<double>() * Eigen::Vector3d(center_point(0), center_point(1), 0.0)).head(2);

    auto& reference_point = current_tracking_object.l_shape_feature.reference_point;
    reference_point = (tf.cast<double>() * Eigen::Vector3d(reference_point(0), reference_point(1), 0.0)).head(2);

    auto& shape = current_tracking_object.l_shape_feature.shape;
    Eigen::Vector3d direction = Eigen::Vector3d(std::cos(shape(2)), std::sin(shape(2)), 0.0);
    direction = tf.rotation().cast<double>() * direction;
    shape(2) = std::atan2(direction(1), direction(0));
  }

  // tail point feature
  {
    auto& tail_center_point = current_tracking_object.tail_center_feature.tail_center_point;
    tail_center_point = (tf.cast<double>() * Eigen::Vector3d(tail_center_point(0), tail_center_point(1), 0.0)).head(2);

    auto& edge_center_points = current_tracking_object.tail_center_feature.edge_center_points;
    for (int i = 0; i < edge_center_points.cols(); ++i) {
      const Eigen::Vector3d point(edge_center_points(0, i), edge_center_points(1, i), 0.0);
      edge_center_points.col(i) = (tf.cast<double>() * point).head(2);
    }
  }

  // tracker method
  tracker_method_ptr->TransformToCurrent(tf);

  // 以下参数为绝对坐标系下的参数，不需要进行变换
  {
    // history object distance
    while (history_object_distance_.size() > 5) {
      history_object_distance_.pop_front();
    }

    // history object velocity
    while (history_object_velocity_.size() > 10) {
      history_object_velocity_.pop_back();
    }
  }
}

MotionDirection Tracklet::checkObjectMotionDirection(const std::deque<float>& distances) {
  if (distances.size() <= 1UL) return MotionDirection::UNKNOWN;
  const float sum_distance = std::accumulate(distances.begin(), distances.end(), 0.0F);
  const float mean_distance = sum_distance / distances.size();
  if (distances.back() > mean_distance) {
    return MotionDirection::DEPARTURE;
  } else {
    return MotionDirection::ARRIVAL;
  }
}

float Tracklet::checkVelocityConfidence(const std::deque<std::pair<double, Eigen::Vector3f>>& velocities) {
  if (velocities.size() <= 2UL) return 0.0F;
  const float v0 = velocities[0].second.head(2).norm();
  const float v1 = velocities[1].second.head(2).norm();
  const float v2 = velocities[2].second.head(2).norm();
  const double dt0 = velocities[0].first - velocities[1].first;
  const double dt1 = velocities[1].first - velocities[2].first;
  const double dt = (velocities[0].first - velocities[2].first) * 0.5;
  const float a0 = (v0 - v1) / dt0;
  const float a1 = (v1 - v2) / dt1;
  const float jerk_abs = std::abs((a0 - a1) / dt);

  float score = 0.0F;
  if (jerk_abs < 8.0F) {
    score = 1.0F;
  } else if (jerk_abs > 80.0F) {
    score = 0.0F;
  } else {
    score = 1.0F - (jerk_abs - 8.0F) / 80.0F;
  }

  return score;
}

TRUNK_PERCEPTION_LIB_NAMESPACE_END