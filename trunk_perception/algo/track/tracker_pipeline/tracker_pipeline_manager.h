/**
 * @file tracker_manager.h
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-08-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/algo/track/tracker_pipeline/tracker_pipeline_interface.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

class TrackerPipelineManager {
 public:
  TrackerPipelineManager() = default;
  ~TrackerPipelineManager() = default;

  static std::unique_ptr<TrackerPipelineInterface> Create(const std::string name);
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END
