#pragma once

#include <yaml-cpp/yaml.h>
#include <any>
#include <cstdint>

#include "tju_local_planning/common/macros.h"


#include "tju_local_planning/common/types/pose_point.h"
#include "tju_local_planning/common/types/trajectory_points.h"
#include "tju_local_planning/common/types/odometry.h"
#include "tju_local_planning/common/types/objects.h"
#include "tju_local_planning/common/types/target_pose.h"
#include "tju_local_planning/common/types/cross_goal.h"
#include "tju_local_planning/common/types/trajectory_state.h"
#include "tju_local_planning/common/types/ogm_points.h"
#include "tju_local_planning/common/types/car_ori_interface.h"


TJU_LOCAL_PLANNING_LIB_APP_NAMESPACE_BEGIN
using namespace TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE;

/**
 * @brief app base class
 * @details 为所有感知应用的外部调用提供统一的接口
 * @details 应用内部通过DataManager获取数据
 */
class AppBase {
 public:
  AppBase() = default;
  virtual ~AppBase() = default;

  virtual std::uint32_t Init(const YAML::Node& config) = 0;
  virtual std::uint32_t Run(const CrossGoal& cross_goal,
    const TrajectoryPoints& global_trajectory,
    const CarOriInterface& car_ori_data,
    const TargetPose& align_target,
    const Objects& obstacles,
    const OgmPoints& og_points,
    const Odometry& fusion_location) = 0;
  virtual std::any GetData(const std::string& key) = 0;
};

TJU_LOCAL_PLANNING_LIB_APP_NAMESPACE_END
