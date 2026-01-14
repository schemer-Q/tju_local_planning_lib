#pragma once

#include <stdint.h>
#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/odometry.h"
#include "tju_local_planning/common/types/header.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

struct TrajectoryState {
  public:
   //uint64_t timestamp;        // 时间戳
   Header header;              // 头部信息
   Odometry real_time_point;  // 实时点位置
   uint8_t real_sensor_on_flag;      // 实时感知开关
   uint8_t real_forklift_up_flag;    // 实时抬臂开关
   uint8_t real_arrive_goal_pose;    // 是否到达目标点
   uint8_t idx_in_global;            // 在全局轨迹的位置
   
   typedef std::shared_ptr<TrajectoryState> Ptr;
   typedef std::shared_ptr<const TrajectoryState> ConstPtr;
 };

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END
