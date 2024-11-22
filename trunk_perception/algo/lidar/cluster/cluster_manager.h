/**
 * @file cluster_manager.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/algo/lidar/cluster/bev_ccl_cluster/bev_ccl_cluster.h"
#include "trunk_perception/algo/lidar/cluster/cluster_base.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

class ClusterManager {
 public:
  ClusterManager() = default;
  ~ClusterManager() = default;

  static std::shared_ptr<ClusterBase> Create(const std::string& name);
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END