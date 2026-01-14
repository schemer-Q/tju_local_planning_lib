#include "tju_local_planning/app/local_planning_QuinticPolynomialPlanner/local_planner_manager.h"
#include "tju_local_planning/app/local_planning_QuinticPolynomialPlanner/local_planner_manager.h"
#include "tju_local_planning/common/error/code.hpp"
#include <iostream>


TJU_LOCAL_PLANNING_LIB_APP_NAMESPACE_BEGIN

using namespace TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE;

// 初始化静态成员变量
std::unordered_map<std::string, std::function<std::shared_ptr<ForkLiftLocalPlannerBase>()>> 
    ForkLiftLocalPlannerManager::planner_factories_;

void ForkLiftLocalPlannerManager::RegisterPlanners() {
  if (planner_factories_.empty()) {
    // 注册QuinticPolynomial规划器，直接创建ForkLiftLocalPlannerBase实例
    planner_factories_["quinticpolynomial"] = []() {
      return std::make_shared<ForkLiftLocalPlannerBase>();
    };
    planner_factories_["quintic_polynomial"] = planner_factories_["quinticpolynomial"];
  }
}

std::shared_ptr<ForkLiftLocalPlannerBase> ForkLiftLocalPlannerManager::Create(const std::string& file) {
  try {
    YAML::Node config = YAML::LoadFile(file);
    return Create(config);
  } catch (const std::exception& e) {
    std::cerr << "ForkLiftLocalPlannerManager::Create: Failed to load config file: " << e.what() << std::endl;
    // return ErrorCode::YAML_CONFIG_ERROR;
    return nullptr;

  }
}

std::shared_ptr<ForkLiftLocalPlannerBase> ForkLiftLocalPlannerManager::Create(const YAML::Node& config) {
  if (!config["Type"].IsDefined() || !config["Config"].IsDefined()) {
    std::cerr << "ForkLiftLocalPlannerManager::Create: config is invalid" << std::endl;
    // return ErrorCode::YAML_CONFIG_ERROR;
    return nullptr;

  }

  // 确保规划器已注册
  RegisterPlanners();

  std::string planner_type = config["Type"].as<std::string>();
  // 转为小写以实现不区分大小写的查找
  std::transform(planner_type.begin(), planner_type.end(), planner_type.begin(), 
                [](unsigned char c){ return std::tolower(c); });

  // 从注册表中查找规划器工厂
  auto it = planner_factories_.find(planner_type);
  if (it == planner_factories_.end()) {
    std::cerr << "ForkLiftLocalPlannerManager::Create: local planner type not supported: " 
           << config["Type"].as<std::string>() << std::endl;
    // return ErrorCode::YAML_CONFIG_ERROR;
    return nullptr;

  }

  // 创建规划器实例
  auto local_planner = it->second();
  if (!local_planner) {
    std::cerr << "ForkLiftLocalPlannerManager::Create: failed to create planner instance" << std::endl;
    // return ErrorCode::UNINITIALIZED;
    return nullptr;

  }

  // 初始化规划器
  auto ret = local_planner->Init(config);
  if (ret != ErrorCode::SUCCESS) {
    std::cerr << "ForkLiftLocalPlannerManager::Create: init local planner failed, error code: " << ret << std::endl;
    // return ErrorCode::UNINITIALIZED;
    return nullptr;
  }

  return local_planner;
}


TJU_LOCAL_PLANNING_LIB_APP_NAMESPACE_END
