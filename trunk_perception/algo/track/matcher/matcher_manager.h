/**
 * @file matcher_manager.h
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "matcher_base.h"
#include "one_stage_matcher.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

class MatcherManager {
 public:
  MatcherManager() = default;
  ~MatcherManager() = default;

  static std::shared_ptr<MatcherBase> CreateMatcher(const std::string matcher_name);
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END