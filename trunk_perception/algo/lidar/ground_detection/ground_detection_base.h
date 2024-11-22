/**
 * @file cluster_base.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-10-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/point.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

class GroundDetectionBase {
 public:
  GroundDetectionBase() = default;
  virtual ~GroundDetectionBase() = default;

  /**
   * @brief init param
   *
   * @param config yaml node
   * @return int
   */
  virtual int init(const YAML::Node& config) = 0;

  /**
   * @brief process pipeline
   *
   * @param cloud_in input point cloud
   * @return int
   */
  virtual int process(const PointCloudConstPtr& cloud_in) = 0;

  /**
   * @brief set point cloud transform matrix
   *
   * @param tf transform matrix
   */
  virtual void setTF(const Eigen::Isometry3f& tf) = 0;

  /**
   * @brief get no ground cloud
   *
   * @return PointCloudPtr
   */
  virtual PointCloudPtr getNoGroundCloud() = 0;

  /**
   * @brief get ground cloud
   *
   * @return PointCloudPtr
   */
  virtual PointCloudPtr getGroundCloud() = 0;

  /**
   * @brief get ground params object
   *
   * @return std::vector<float>
   */
  virtual std::vector<float> getGroundParams() = 0;
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END