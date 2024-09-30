/**
 * @file distance_measurement.h
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include "trunk_perception/algo/track/tracklet.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

struct ObjectDistanceMeasurementParams {
  std::string method = "";     // NORMAL_DISTANCE、FUSION_DISTANCE
  float max_distance = 5.0F;   // 前后帧最大距离(m)
  float max_velocity = 50.0F;  // 障碍物最大速度(m/s)

  // location、direction、bbox size、point num、centroid、iou、box tail
  std::vector<float> weight = {0.3F, 0.1F, 0.1F, 0.1F, 0.01F, 0.1F, 0.2F};
};

class ObjectDistanceMeasurement {
 public:
  ObjectDistanceMeasurement() = default;
  ~ObjectDistanceMeasurement() = default;

  /**
   * @brief object distance measurement init
   *
   * @param config yaml node
   * @return int
   */
  int Init(const YAML::Node& config);

  /**
   * @brief compute track and object measurement distance
   *
   * @param track tracking object
   * @param object current detection object
   * @return float measurement distance
   */
  float ComputeDistance(const Tracklet& track, const Object& object);

 private:
  /**
   * @brief compute track and object fusion measurement distance
   *
   * @param track tracking object
   * @param object current detection object
   * @return float fusion measurement distance
   */
  float fusionDistance(const Tracklet& track, const Object& object);

  /**
   * @brief compute track and object normal measurement distance
   *
   * @param track tracking object
   * @param object current detection object
   * @return float fusion measurement distance
   */
  float normalDistance(const Tracklet& track, const Object& object);

 private:
  ObjectDistanceMeasurementParams params_;
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END