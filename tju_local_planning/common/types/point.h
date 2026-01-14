#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>

#include "tju_local_planning/common/macros.h"

struct PointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (float, time, time)
)
// clang-format on

typedef PointXYZIRT PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef std::shared_ptr<PointCloudT> PointCloudPtr;  // 注意：这里使用的是std::shared_ptr，而不是boost::shared_ptr
typedef std::shared_ptr<const PointCloudT>
    PointCloudConstPtr;  // 注意：这里使用的是std::shared_ptr，而不是boost::shared_ptr
