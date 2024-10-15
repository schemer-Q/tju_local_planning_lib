/**
 * @file tail_middle_tracker_cv.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-10-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <memory>

#include "trunk_perception/algo/track/common/geometric_algo.h"
#include "trunk_perception/algo/track/kalman_filter/linear_kalman_filter.h"
#include "trunk_perception/algo/track/tracker_method/tracker_method_base.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

class TailCenterTrackerCV : virtual public TrackerMethodBase {
 public:
  TailCenterTrackerCV() = default;
  ~TailCenterTrackerCV() override = default;

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
   * @brief Get the Track Model object
   *
   * @param object object tracked
   */
  void getTrackModel(Object& object);

 private:
  std::shared_ptr<LinearKalmanFilter> motion_filter_ = nullptr;  // 运动状态滤波器
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END