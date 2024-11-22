/**
 * @file center_distance.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-11-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/algo/track/matcher/distance_measurement/distance_base.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

using common::BoundingBox;

struct CenterDistanceParams {
  float max_distance = 5.0F;   // 前后帧最大距离(m)
  float max_velocity = 50.0F;  // 障碍物最大速度(m/s)
};

class CenterDistance : virtual public DistanceBase {
 public:
  CenterDistance() = default;
  ~CenterDistance() override = default;

  /**
   * @brief object distance measurement init
   *
   * @param config yaml node
   * @return int
   */
  int Init(const YAML::Node& config) override;

  /**
   * @brief compute track and object measurement distance
   *
   * @param track tracking object
   * @param object current detection object
   * @return float measurement distance
   */
  float ComputeDistance(const Tracklet& track, const Object& object) override;

 private:
  float getDistanceError(const BoundingBox& track_bbox, const BoundingBox& detect_bbox);

 private:
  CenterDistanceParams params_;
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END