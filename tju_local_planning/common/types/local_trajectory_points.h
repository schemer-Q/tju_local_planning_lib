#pragma once

#include <memory>
#include <vector>

#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/pose_point.h"
#include "tju_local_planning/common/types/header.h"
TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN
 //局部规划轨迹使用
 
struct LocalTrajectory {
 public:
  // 对应std_msgs/Header header
  //uint64_t timestamp;
  Header header;
  
  std::vector<PosePoint> trajectory;  // vehicle trajectory 
  uint8_t task_type;      // 0正常跟踪，1局部对准
  int64_t points_cnt;     // 轨迹点个数
  double step_length;     // 两点间距
  int64_t replan_counter; // 规划次数
  uint8_t plan_state;     // 0:未规划完成 1：规划完成

  typedef std::shared_ptr<LocalTrajectory> Ptr;
  typedef std::shared_ptr<const LocalTrajectory> ConstPtr;
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END
