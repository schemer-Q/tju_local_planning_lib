// 用于构建代价地图的数据结构struct
#pragma once
#include <memory>
#include <vector>

#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/header.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

// 点结构（参考上游点云常见字段）
struct PointXYZI {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  float intensity = 0.0f;    // 可选回波强度
  uint32_t ring = 0;         // 激光线束索引（若有）
  double time = 0.0;         // 点相对/绝对时间（秒），用于去畸变
};

using PointCloud = std::vector<PointXYZI>;

// 传感器/里程计位姿，用于将点云投影到地图坐标系
struct Pose {
  Header header;             // 包含时间戳与 frame_id
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  std::vector<double> covariance; // 可选：6x6 行主序协方差

  typedef std::shared_ptr<Pose> Ptr;
  typedef std::shared_ptr<const Pose> ConstPtr;
};

// 传感器外参（从传感器到车体/地图的刚体变换）
struct SensorExtrinsics {
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0; // 四元数

  typedef std::shared_ptr<SensorExtrinsics> Ptr;
  typedef std::shared_ptr<const SensorExtrinsics> ConstPtr;
};

// 点云消息封装（与上游消息解耦后使用）
struct PointCloudMsg {
  Header header;
  PointCloud points;

  typedef std::shared_ptr<PointCloudMsg> Ptr;
  typedef std::shared_ptr<const PointCloudMsg> ConstPtr;
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END