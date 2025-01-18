#pragma once

#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>

#include "trunk_perception/algo/track/common/id_manager.h"
#include "trunk_perception/common/data_manager/data_wrapper/ld_frame.h"
#include "trunk_perception/common/types/lane.h"
#include "trunk_perception/app/lane_detection_post/algo/lane_quality.h"
#include "trunk_perception/app/lane_detection_post/algo/lane_tracklet.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_post{
  
using common::LaneLineVision;
using common::LDFrame;

class BaseLaneTracker {
 public:
  using LaneTrackletPtr = std::shared_ptr<LaneTracklet>;
  using LaneQualityEvaluatorPtr = std::shared_ptr<LaneQualityEvaluator>;

  BaseLaneTracker() = default;
  virtual ~BaseLaneTracker() = default;

  virtual int Init(const YAML::Node& config) = 0;

  virtual int Run(const std::vector<LaneLineVision>& lanelines_detected, 
                  const Eigen::Matrix4d& pose, 
                  const double timestamp,
                  std::vector<LaneLineVision>& lanelines_tracked) = 0;

};

} // namespace ld_post

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
