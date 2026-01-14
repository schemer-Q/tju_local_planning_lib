#pragma once

#include <memory>
#include <vector>

#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/pose_point.h"
#include "tju_local_planning/common/types/header.h"
TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN
//全局规划轨迹使用的轨迹

struct TrajectoryPoints {
 public:
  // 对应std_msgs/Header header
  //uint64_t timestamp;
  Header header;
  
  std::vector<PosePoint> trajectory;  // vehicle trajectory 
  //uint8_t task_type;     // 0:正常跟踪 1:局部对准
  int64_t points_cnt;    // 轨迹点个数
  double step_length;    // 两点间距
  int64_t replan_counter;// 规划次数 第几次做的规划
  uint8_t plan_state;    // 0:未规划完成 1：规划完成
  

  typedef std::shared_ptr<TrajectoryPoints> Ptr;
  typedef std::shared_ptr<const TrajectoryPoints> ConstPtr;
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END
