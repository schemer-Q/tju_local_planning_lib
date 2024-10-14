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

#include "trunk_perception/algo/track/common/tracklet.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

void Tracklet::Predict(const double timestamp) {
  const double dt = timestamp - current_tracking_object.timestamp;
  tracker_method_ptr->Predict(dt, current_tracking_object);

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

  current_tracking_object.consecutive_lost += 1;
  current_tracking_object.timestamp = timestamp;
}

void Tracklet::Update(const Object& object) {
  tracker_method_ptr->Update(object, current_tracking_object);

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
  current_tracking_object.convex_polygon = object.convex_polygon;
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

  // tracker method
  tracker_method_ptr->TransformToCurrent(tf);
}

TRUNK_PERCEPTION_LIB_NAMESPACE_END