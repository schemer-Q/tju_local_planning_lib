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

#include "trunk_perception/algo/track/tracklet.h"
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

typedef std::pair<size_t, size_t> TrackObjectPair;

struct MatcherOptions {
  float cost_thresh = 0.99F;
  float bound_value = 1.0F;
};

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
   * @param objects_tracked tracking objects
   * @param objects_detected detection objects
   * @param assignments pair assigned tracking objects and detection objects
   * @param unassigned_tracks unassigned tracking objects
   * @param unassigned_objects unassigned detection objects
   * @return int
   */
  virtual int Match(const std::vector<Tracklet>& objects_tracked, const std::vector<Object>& objects_detected,
                    std::vector<TrackObjectPair>* assignments, std::vector<size_t>* unassigned_tracks,
                    std::vector<size_t>* unassigned_objects) = 0;
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END