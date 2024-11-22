/**
 * @file cluster_base.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/point.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

class ClusterBase {
 public:
  ClusterBase() = default;
  virtual ~ClusterBase() = default;

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
   * @brief get the clusters cloud
   *
   * @return std::vector<typename std::shared_ptr<pcl::PointCloud<T>>>
   */
  virtual std::vector<PointCloudConstPtr> getClustersCloud() = 0;
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END