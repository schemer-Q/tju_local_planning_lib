/**
 * @file fusion_distance.h
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

struct FusionDistanceParams {
  // location、direction、bbox size、point num、centroid、iou、box tail
  std::vector<float> weight = {0.3F, 0.1F, 0.1F, 0.1F, 0.01F, 0.1F, 0.2F};
};

class FusionDistance : virtual public DistanceBase {
 public:
  FusionDistance() = default;
  ~FusionDistance() override = default;

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
  /**
   * @brief compute location distance for given track and object
   *
   * @param track current tracking object
   * @param object current detection object
   * @param time_diff time diff from current tracking object to current detection object
   * @return float distance result
   */
  float LocationDistance(const Tracklet& track, const Object& object, const double time_diff);

  /**
   * @brief compute direction distance for given track and object
   *
   * @param track current tracking object
   * @param object current detection object
   * @param time_diff time diff from current tracking object to current detection object
   * @return float distance result
   */
  float DirectionDistance(const Tracklet& track, const Object& object, const double time_diff);

  /**
   * @brief compute bbox size distance for given track and object
   *
   * @param track current tracking object
   * @param object current detection object
   * @param time_diff time diff from current tracking object to current detection object
   * @return float distance result
   */
  float BboxSizeDistance(const Tracklet& track, const Object& object, const double time_diff);

  /**
   * @brief compute point num distance for given track and object
   *
   * @param track current tracking object
   * @param object current detection object
   * @param time_diff time diff from current tracking object to current detection object
   * @return float distance result
   */
  float PointNumDistance(const Tracklet& track, const Object& object, const double time_diff);

  /**
   * @brief compute centroid shift distance for given track and object
   *
   * @param track current tracking object
   * @param object current detection object
   * @param time_diff time diff from current tracking object to current detection object
   * @return float distance result
   */
  float CentroidShiftDistance(const Tracklet& track, const Object& object, const double time_diff);

  /**
   * @brief compute bbox iou distance for given track and object
   *
   * @param track current tracking object
   * @param object current detection object
   * @param time_diff time diff from current tracking object to current detection object
   * @return float distance result
   */
  float BboxIouDistance(const Tracklet& track, const Object& object, const double time_diff);

  /**
   * @brief compute box tail distance for given track and object
   *
   * @param track current tracking object
   * @param object current detection object
   * @param time_diff time diff from current tracking object to current detection object
   * @return float distance result
   */
  float BboxTailDistance(const Tracklet& track, const Object& object, const double time_diff);

 private:
  FusionDistanceParams params_;
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END