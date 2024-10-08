/**
 * @file tracker_pipeline_interface.h
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include "trunk_perception/algo/track/common/id_manager.h"
#include "trunk_perception/common/data_manager/data_wrapper/od_lidar_frame.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

using common::OdLidarFrame;

class TrackerPipelineInterface {
 public:
  TrackerPipelineInterface() = default;
  virtual ~TrackerPipelineInterface() = default;

  /**
   * @brief lidar tracker interface init
   *
   * @param config
   * @return int
   */
  virtual int Init(const YAML::Node& config) = 0;

  /**
   * @brief track pipeline
   *
   * @param frame object detection lidar frame
   * @return int
   */
  virtual int Track(std::shared_ptr<OdLidarFrame>& frame) = 0;

  /**
   * @brief set id manager
   *
   * @param id_manager_ptr tracker id manager pointer
   */
  virtual void SetIDManager(const IDManagerPtr& id_manager_ptr) = 0;
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END
