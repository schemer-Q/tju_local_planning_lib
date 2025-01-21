/**
 * @file track_param.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2025-01-15
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>
#include <string>

#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

struct SimpleTrackParams {
  int min_lifetime_output = 3;
  int max_consecutive_lost_num = 5;
  int min_consecutive_valid_num = 3;
  bool nms_by_polygon = false;
  bool predict_by_velocity = false;
  int trigger_predict_count = 10;
  float max_velocity = 50.0F;

  std::string traker_method = "";
  YAML::Node traker_params;
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END