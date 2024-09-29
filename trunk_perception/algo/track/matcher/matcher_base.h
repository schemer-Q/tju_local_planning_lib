/**
 * @file matcher_base.h
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include "../tracklet.h"
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

class MatcherBase {
 public:
  MatcherBase() = default;
  virtual ~MatcherBase() = default;

  /**
   * @brief matcher init
   *
   * @param config yaml node
   * @return int
   */
  virtual int Init(const YAML::Node& config) = 0;

  /**
   * @brief match objects detected and objects tracked
   *
   * @param objects_detected detection objects
   * @param objects_tracked tracking objects
   * @param assignment match result
   * @return int
   */
  virtual int Match(const std::vector<Tracklet>& objects_tracked, const std::vector<Object>& objects_detected,
                    std::vector<int>& assignment) = 0;
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END