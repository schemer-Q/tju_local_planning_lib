/**
 * @file bev_lane_post_impl.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 基于BEV的车道线后处理实现
 * @version 0.1
 * @date 2024-10-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "trunk_perception/app/lane_detection_post/algo/tracker/lane_point_tracker.h"
#include "trunk_perception/app/lane_detection_post/ld_post_base.h"
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

class BevLanePostImpl : public LdPostBase {
 public:
  BevLanePostImpl() = default;
  ~BevLanePostImpl() override;

  std::uint32_t Init(const YAML::Node& config) override;
  std::uint32_t Run(const double& ts) override;
  std::any GetData(const std::string& key) override;

 private:
  std::shared_ptr<ld_post::LanePointTracker> lane_tracker_ptr_;
  };

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
