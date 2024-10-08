/**
 * @file tracker_pipeline_manager.cpp
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "trunk_perception/algo/track/tracker_pipeline/tracker_pipeline_manager.h"
#include "trunk_perception/algo/track/tracker_pipeline/simple_track.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

std::unique_ptr<TrackerPipelineInterface> TrackerPipelineManager::Create(const std::string name) {
  if (name == "SimpleTrack") {
    return std::make_unique<SimpleTrack>();
  } else {
    TFATAL << "[TrackerPipelineManager] name: " << name << " not supported!";
  }

  return nullptr;
}

TRUNK_PERCEPTION_LIB_NAMESPACE_END