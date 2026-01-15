#pragma once

#include <yaml-cpp/yaml.h>
#include <any>
#include <string>

#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/pose_point.h"

TJU_LOCAL_PLANNING_LIB_NAMESPACE_BEGIN

/**
 * @brief 局部轨迹规划器基类
 */
class LocalTrajectoryPlannerBase {
 public:
  LocalTrajectoryPlannerBase() = default;
  virtual ~LocalTrajectoryPlannerBase() = default;

  /**
   * @brief 初始化参数
   *
   * @param config yaml配置节点
   * @return int 0表示成功
   */
  virtual int init(const YAML::Node& config) = 0;

  /**
   * @brief 处理路径规划
   *
   * @return int 0表示成功
   */
  virtual int process() = 0;
  virtual int process(const tju::local_planning::common::PosePoint& current_pose,const tju::local_planning::common::PosePoint& target_pose)=0;

  /**
   * @brief 获取处理结果数据
   *
   * @param key 数据键名
   * @return std::any 返回的数据
   */
  virtual std::any GetData(const std::string& key) = 0;
};

TJU_LOCAL_PLANNING_LIB_NAMESPACE_END
