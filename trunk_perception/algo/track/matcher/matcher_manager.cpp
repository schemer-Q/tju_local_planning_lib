/**
 * @file matcher_manager.cpp
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "matcher_manager.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

std::shared_ptr<MatcherBase> MatcherManager::CreateMatcher(const std::string matcher_name) {
  if (matcher_name == "OneStage") {
    return std::make_shared<OneStageMatcher>();
  } else {
    TFATAL << "[MatcherManager] matcher_name: " << matcher_name << " not supported!";
  }

  return nullptr;
}

TRUNK_PERCEPTION_LIB_NAMESPACE_END