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

#include "trunk_perception/algo/lidar/ground_detection/ground_detection_base.h"
#include "trunk_perception/algo/lidar/ground_detection/jcp_ground_detection/jcp_ground_detection.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

class GroundDetectionManager {
 public:
  GroundDetectionManager() = default;
  ~GroundDetectionManager() = default;

  static std::shared_ptr<GroundDetectionBase> Create(const std::string& name);
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END