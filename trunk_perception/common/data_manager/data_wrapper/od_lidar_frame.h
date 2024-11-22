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

#include "trunk_perception/algo/track/common/id_manager.h"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/object.h"
#include "trunk_perception/common/types/point.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

/**
 * @brief 基于激光雷达的物体检测帧, 用于不同app线程之间共享数据，被DataManager管理
 *
 */
struct OdLidarFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp = 0.0;                                ///< 时间戳, 单位为秒
  PointCloudConstPtr pointcloud = nullptr;               ///< 点云数据
  PointCloudConstPtr noground_cloud = nullptr;           ///< 非地面点云数据
  std::vector<Object> detected_objects;                  ///< 当前帧障碍物检测列表
  Eigen::Isometry3f tf = Eigen::Isometry3f::Identity();  ///< 前后帧的变换位姿
  std::vector<Object> tracked_objects;                   ///< 跟踪后的障碍物列表
  std::vector<PointCloudConstPtr> clusters_cloud;        ///< 聚类点云列表
  std::vector<Object> cluster_objects;                   ///< 聚类障碍物列表
  IDManagerPtr id_manager_ptr = nullptr;                 ///< 障碍物跟踪id poll
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END