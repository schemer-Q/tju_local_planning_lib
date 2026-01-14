#include "tju_local_planning/algo/local_planning/QuinticPolynomialPlanner/local_trajectory_planner_manager.h"
#include "tju_local_planning/algo/local_planning/QuinticPolynomialPlanner/QuinticPolynomial/quintic_polynomial_planner.h"
#include "tju_local_planning/algo/local_planning/QuinticPolynomialPlanner/HybridAStar/HybridAStar.h"
#include "tju_local_planning/algo/local_planning/QuinticPolynomialPlanner/hybrid_astar/hybrid_a_star_bridge.h"
#include <iostream>

TJU_LOCAL_PLANNING_LIB_NAMESPACE_BEGIN


std::shared_ptr<LocalTrajectoryPlannerBase> LocalTrajectoryPlannerManager::Create(const std::string& name) {
  // 不区分大小写比较
  if (name == "QuinticPolynomial") {
    return std::make_shared<QuinticPolynomialPlanner>();
  }
  if (name == "HybridAStar") {
    return std::make_shared<HybridAStar>();
  }
  if (name == "HybridAStarBridge") {
    return std::make_shared<HybridAStarBridge>();
  }
  
  return nullptr;
}

TJU_LOCAL_PLANNING_LIB_NAMESPACE_END
