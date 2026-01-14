#pragma once

#include <memory>
#include <string>

#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/pose_point.h"
#include "tju_local_planning/common/types/header.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

struct CrossGoal {
 public:
  // 对应std_msgs/Header header
  //uint64_t timestamp;
  Header header;
  
  std::string vin;     // 车辆vin码
  uint8_t work_state;  // 0: 未到达任务终点; 1: 到达任务终点
  uint8_t tra_track_state; // 0: 轨迹跟踪状态; 1: 车辆当前轨迹跟踪已完成
  PosePoint target_point;  // 目标点
  PosePoint end_point;     // 实际终点

  typedef std::shared_ptr<CrossGoal> Ptr;
  typedef std::shared_ptr<const CrossGoal> ConstPtr;
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END
