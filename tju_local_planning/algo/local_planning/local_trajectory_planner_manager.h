#pragma once

#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>

#include "tju_local_planning/algo/local_planning/local_trajectory_planner_base.h"
#include "tju_local_planning/common/macros.h"

TJU_LOCAL_PLANNING_LIB_NAMESPACE_BEGIN


/**
 * @brief 局部轨迹规划器管理类
 */
class LocalTrajectoryPlannerManager {
 public:
  LocalTrajectoryPlannerManager() = default;
  ~LocalTrajectoryPlannerManager() = default;

  /**
   * @brief 创建局部规划算法
   *
   * @param name 算法名称
   * @return std::shared_ptr<LocalTrajectoryPlannerBase>
   */
  static std::shared_ptr<LocalTrajectoryPlannerBase> Create(const std::string& name);
};
TJU_LOCAL_PLANNING_LIB_NAMESPACE_END
