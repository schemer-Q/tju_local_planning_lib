/**
 * @file debug_data.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 定义一些debug所需的数据结构
 * @version 0.1
 * @date 2024-10-31
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "trunk_perception/app/target_fusion/data_associate/tracker_objects_match.h"
#include "trunk_perception/app/target_fusion/data_fusion/tracker.h"
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

struct AssociateDebugData {
  std::vector<TrackerPtr> new_trackers;
  std::vector<TrackerPtr> stable_trackers;
  std::vector<TrackerPtr> lost_trackers;

  AssociationResult new_tracker_lidar_association_result;
  AssociationResult stable_tracker_lidar_association_result;
  AssociationResult lost_tracker_lidar_association_result;

  AssociationResult new_tracker_radar_association_result;
  AssociationResult stable_tracker_radar_association_result;
  AssociationResult lost_tracker_radar_association_result;

  AssociationResult new_tracker_front_vision_association_result;
  AssociationResult stable_tracker_front_vision_association_result;
  AssociationResult lost_tracker_front_vision_association_result;

  AssociationResult new_tracker_corner_radar_1_association_result;
  AssociationResult stable_tracker_corner_radar_1_association_result;
  AssociationResult lost_tracker_corner_radar_1_association_result;

  AssociationResult new_tracker_corner_radar_5_association_result;
  AssociationResult stable_tracker_corner_radar_5_association_result;
  AssociationResult lost_tracker_corner_radar_5_association_result;

  AssociationResult new_tracker_corner_radar_7_association_result;
  AssociationResult stable_tracker_corner_radar_7_association_result;
  AssociationResult lost_tracker_corner_radar_7_association_result;

  AssociationResult new_tracker_corner_radar_11_association_result;
  AssociationResult stable_tracker_corner_radar_11_association_result;
  AssociationResult lost_tracker_corner_radar_11_association_result;

  typedef std::shared_ptr<AssociateDebugData> Ptr;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END