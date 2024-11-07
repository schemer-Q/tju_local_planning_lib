/**
 * @file target_fusion_frame.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 目标融合帧
 * @version 0.1
 * @date 2024-10-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <Eigen/Dense>
#include <cstddef>

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/fused_object.h"
#include "trunk_perception/common/types/odometry.h"
#include "trunk_perception/common/types/radar_ars430.h"
#include "trunk_perception/common/types/radar_cr5tp.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

/**
 * @brief 目标后融合数据帧
 * @details 如无特殊说明，均为车体系下坐标
 */
struct TargetFusionFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp = 0.0;  ///< 后融合任务启动时的系统时间

  // 来自激光雷达检测管线的数据
  double lidar_timestamp = 0.0;
  std::vector<LidarMeasureFrame::Ptr> lidar_tracked_objects;        ///< 激光雷达检测到的目标
  std::vector<LidarMeasureFrame::Ptr> lidar_tracked_objects_local;  ///< 局部坐标下的激光雷达检测到的目标

  // 来自毫米波雷达检测管线的数据
  double front_radar_timestamp = 0.0;
  std::vector<ars430::RadarMeasureFrame::Ptr> front_radar_objects;              ///< 毫米波雷达检测到的目标
  std::vector<ars430::RadarMeasureFrame::Ptr> front_radar_objects_compensated;  ///< 补偿到激光雷达时间的目标, 车体系下
  std::vector<ars430::RadarMeasureFrame::Ptr> front_radar_objects_local;  ///< 局部坐标下的毫米波雷达检测到的目标

  // Odometry数据
  Odometry::Ptr odometry_lidar_ptr = nullptr;
  Odometry::Ptr odometry_front_radar_ptr = nullptr;
  Odometry::Ptr odometry_ts_ptr = nullptr;

  // 未分配的观测数据
  std::vector<size_t> unassigned_lidar_objects_;
  std::vector<size_t> unassigned_front_radar_objects_;

  // 融合后的目标
  std::vector<FusedObject::Ptr> fused_objects;
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END