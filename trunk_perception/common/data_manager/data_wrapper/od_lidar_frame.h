/**
 * @file od_lidar_frame.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 基于激光雷达的物体检测帧
 * @version 0.1
 * @date 2024-09-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/point.h"
#include "trunk_perception/common/types/object.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

/**
 * @brief 基于激光雷达的物体检测帧, 用于不同app线程之间共享数据，被DataManager管理
 *
 */
struct OdLidarFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp = 0.0; ///< 时间戳, 单位为秒
  PointCloudConstPtr pointcloud = nullptr; ///< 点云数据
  std::vector<Object> detected_objects;
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END