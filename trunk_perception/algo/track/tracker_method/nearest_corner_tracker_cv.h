/**
 * @file nearest_corner_tracker_cv.h
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <memory>

#include "trunk_perception/algo/track/kalman_filter/linear_kalman_filter.h"
#include "trunk_perception/algo/track/tracker_method/tracker_method_base.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

enum SwitchDirection { NO_SWITCH, CLOCKWISE, COUNTER_CLOCKWISE };

class NearestCornerTrackerCV : virtual public TrackerMethodBase {
 public:
  NearestCornerTrackerCV() = default;
  ~NearestCornerTrackerCV() override = default;

  /**
   * @brief init tracker method
   *
   * @param config yaml node
   * @param object current detection object
   * @return int
   */
  int Init(const YAML::Node& config, const Object& object) override;

  /**
   * @brief tracker predict
   *
   * @param dt time interval from previous frame to current frame
   * @param object_tracked current tracking object
   */
  void Predict(const double dt, Object& object_tracked) override;

  /**
   * @brief tracker update
   *
   * @param object current detection object
   * @param object_tracked current tracking object
   */
  void Update(const Object& object, Object& object_tracked) override;

  /**
   * @brief transform tracker parameters from previous frame to current frame
   *
   * @param tf transform matrix from previous frame to current frame
   */
  void TransformToCurrent(const Eigen::Isometry3f& tf) override;

 private:
  /**
   * @brief detect whether corner point is switch
   *
   * @param from_angle angle
   * @param to_angle angle
   * @return SwitchDirection
   */
  SwitchDirection detectCornerPointSwitch(const double from_angle, const double to_angle);

  /**
   * @brief Get the Track Model object
   *
   * @param object object tracked
   */
  void getTrackModel(Object& object);

 private:
  bool initialized_ = false;                                     // 初始化标识
  std::shared_ptr<LinearKalmanFilter> motion_filter_ = nullptr;  // 运动状态滤波器
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END