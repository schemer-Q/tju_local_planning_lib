/**
 * @file tracklet.h
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/algo/track/common/track_param.h"
#include "trunk_perception/algo/track/tracker_method/tracker_method_base.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

enum class TrackletState { UNCONFIRMED, CONFIRMED, DEAD };

class Tracklet {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Tracklet(const SimpleTrackParams& param) : params_(param) {}
  ~Tracklet() = default;

  /**
   * @brief tracker predict
   *
   * @param timestamp current frame timestamp
   */
  void Predict(const double timestamp);

  /**
   * @brief tracker update
   *
   * @param object current detection object
   */
  void Update(const Object& object);

  /**
   * @brief transform tracklet parameters from previous frame to current frame
   *
   * @param tf transform matrix from previous frame to current frame
   */
  void TransformToCurrent(const Eigen::Isometry3f& tf);

  /**
   * @brief determine if lifecycle dieout
   *
   * @return true
   * @return false
   */
  inline bool Dieout() const { return state == TrackletState::DEAD; }

 public:
  Object current_tracking_object;
  std::shared_ptr<TrackerMethodBase> tracker_method_ptr = nullptr;
  TrackletState state = TrackletState::UNCONFIRMED;

 private:
  SimpleTrackParams params_;  // track param
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END