#pragma once

#include <yaml-cpp/yaml.h>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "tju_local_planning/app/local_planning_QuinticPolynomialPlanner/forklift_local_planner_base.h"
#include "tju_local_planning/common/macros.h"

TJU_LOCAL_PLANNING_LIB_APP_NAMESPACE_BEGIN


/**
 * @brief 叉车局部规划器管理类
 */
class ForkLiftLocalPlannerManager {
public:
  /**
   * @brief 从配置文件创建规划器
   * 
   * @param file 配置文件路径
   * @return 规划器实例
   */
  static std::shared_ptr<ForkLiftLocalPlannerBase> Create(const std::string& file);
  
  /**
   * @brief 从配置节点创建规划器
   * 
   * @param config 配置节点
   * @return 规划器实例
   */
  static std::shared_ptr<ForkLiftLocalPlannerBase> Create(const YAML::Node& config);

private:
  /**
   * @brief 注册所有支持的规划器
   */
  static void RegisterPlanners();
  
  // 存储规划器工厂函数的映射表
  static std::unordered_map<std::string, std::function<std::shared_ptr<ForkLiftLocalPlannerBase>()>> planner_factories_;
};


TJU_LOCAL_PLANNING_LIB_APP_NAMESPACE_END
