/**
 * @file point.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 点云数据类型
 * @version 0.1
 * @date 2024-09-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>

#include "common/macros.h"

struct PointXYZIRADT {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  uint16_t column;
  float distance;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRADT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (std::uint16_t, column, column)
  (float, distance, distance)
  (float, time, time)
)
// clang-format on

typedef PointXYZIRADT PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef std::shared_ptr<PointCloudT> PointCloudPtr; // 注意：这里使用的是std::shared_ptr，而不是boost::shared_ptr
typedef std::shared_ptr<const PointCloudT> PointCloudConstPtr; // 注意：这里使用的是std::shared_ptr，而不是boost::shared_ptr