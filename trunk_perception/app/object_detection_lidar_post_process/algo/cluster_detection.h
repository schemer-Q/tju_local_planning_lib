/**
 * @file cluster_detection.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-11-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

#include "trunk_perception/algo/lidar/cluster/cluster_manager.h"
#include "trunk_perception/algo/track/tracker_pipeline/tracker_pipeline_interface.h"
#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/types/odometry.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

struct ClusterDetectionParams {
  // 高度滤除参数
  bool height_filter_switch = true;
  float range_x_threshold = 20.0F;
  float height_threshold = 1.5F;

  // 强度滤除参数
  bool intensity_filter_switch = true;
  float low_intensity_threshold = 5.5F;
  float high_intensity_threshold = 10.5F;
  int high_count_threshold = 3;
  float ratio_threshold = 0.4F;

  // merge objects参数
  float dilate_x = 4.0F;
  float dilate_y = 0.6F;
  float overlap_thresh = 0.1F;
};

struct RemoveObjectPointsParams {
  float scale = 5.0F;
  float x_range = 200.0F;
  float y_range = 200.0F;
  float x_offset = 0.0F;
  float y_offset = 100.0F;
  float x_dilate = 0.0F;
  float y_dilate = 0.0F;
};

class ClusterDetection {
 public:
  ClusterDetection() = default;
  ~ClusterDetection() = default;

  /**
   * @brief cluster detector init
   *
   * @param config yaml node
   * @return int
   */
  int Init(const YAML::Node& config);

  /**
   * @brief cluster detector process pipeline
   *
   * @param frame data frame
   * @return int
   */
  int Process(std::shared_ptr<OdLidarFrame>& frame);

 private:
  void removeObjectPoints(const PointCloudConstPtr& cloud, const std::vector<Object>& objects,
                          PointCloudConstPtr& obstacle_cloud);
  void buildObjects(const std::vector<PointCloudConstPtr>& clusters_cloud, std::vector<Object>& objects,
                    const double ts);
  void nms(std::vector<Object>& objects);
  void filterByRules(std::vector<Object>& objects);
  void mergeObjects(const std::vector<Object>& cluster_objects, std::vector<Object>& merged_objects);

 private:
  ClusterDetectionParams params_;
  std::shared_ptr<ClusterBase> cluster_ = nullptr;

  bool remove_object_points_ = false;
  RemoveObjectPointsParams rop_params_;

  bool track_switch_ = false;
  std::unique_ptr<TrackerPipelineInterface> tracker_ = nullptr;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END