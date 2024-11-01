/**
 * @file cluster_manager.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "trunk_perception/algo/lidar/ground_detection/ground_detection_manager.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

template <class T>
std::shared_ptr<GroundDetectionBase<T>> GroundDetectionManager<T>::Create(const std::string& name) {
  if (name == "JCP") {
    return std::make_shared<JCPGroundDetection<T>>();
  } else {
    TFATAL << "[GroundDetectionManager] name: " << name << " not supported!";
  }
  return nullptr;
}

template class GroundDetectionManager<PointT>;

TRUNK_PERCEPTION_LIB_NAMESPACE_END