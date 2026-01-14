#pragma once

#include <memory>

#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/pose_point.h"
#include "tju_local_planning/common/types/header.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

struct TargetPose {
 public:
  // 对应std_msgs/Header header
  //uint64_t timestamp;
  Header header;
  
  PosePoint align_target;  // 车辆最终位姿

  typedef std::shared_ptr<TargetPose> Ptr;
  typedef std::shared_ptr<const TargetPose> ConstPtr;
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END
