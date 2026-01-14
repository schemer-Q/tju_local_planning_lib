#pragma once

#include <memory>
#include <cstdint>

#include "tju_local_planning/common/macros.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

/**
 * @brief 位姿点数据结构
 * @details 用于表示轨迹点的位置、姿态和动力学信息
 */
struct PosePoint {
 public:
  double x;      // trajectory point x
  double y;      // trajectory point y
  double z;      // trajectory point z
  double pitch;  // x轴转动
  double roll;   // y轴转动
  double yaw;    // z轴航向 (原heading)
  double speed;  // 速度 (原velocity)
  double curve;  // 曲率 (原curvature)
  double acc;    // 加速度 (原acceleration)
  uint8_t gear;  // 档位， 0 - P ；1 - R； 2 - N； 3 - D

  // 构造函数，提供默认值初始化
  // PosePoint()
  //     : x(0.0), y(0.0), z(0.0), pitch(0.0), roll(0.0), yaw(0.0),
  //       speed(0.0), curve(0.0), acc(0.0), gear(0) {}  // 默认D档
  
  // 智能指针类型定义
  typedef std::shared_ptr<PosePoint> Ptr;
  typedef std::shared_ptr<const PosePoint> ConstPtr;
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END
