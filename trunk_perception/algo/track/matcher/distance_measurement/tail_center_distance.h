/**
 * @file tail_center_distance.h
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

struct TailCenterDistanceParams {
  float max_distance = 5.0F;             // 前后帧最大距离(m)
  float max_velocity = 50.0F;            // 障碍物最大速度(m/s)
  float max_theta_diff = 0.785F;         // pi/4, heading角度差阈值(rad)
  float max_projection_distance = 2.0F;  // 在heading垂直方向上的投影距离阈值(m)
};

class TailCenterDistance : virtual public DistanceBase {
 public:
  TailCenterDistance() = default;
  ~TailCenterDistance() override = default;

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
  TailCenterDistanceParams params_;
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END