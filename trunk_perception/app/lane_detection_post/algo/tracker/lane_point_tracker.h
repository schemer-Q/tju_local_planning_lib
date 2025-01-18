// lane_point_tracker.h
#pragma once

#include "trunk_perception/app/lane_detection_post/algo/tracker/base_lane_tracker.h"
#include "trunk_perception/app/lane_detection_post/algo/hungarian.h"
#include "trunk_perception/app/lane_detection_post/algo/id_pool.h"
#include "trunk_perception/app/lane_detection_post/algo/lane_tracklet.h"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_post {
        
class LanePointTracker : public BaseLaneTracker {
 public:
  LanePointTracker() = default;
  ~LanePointTracker() override = default;

  int Init(const YAML::Node& config) override;

  int Run(const std::vector<LaneLineVision>& lanelines_detected, 
          const Eigen::Matrix4d& pose, 
          const double timestamp,
          std::vector<LaneLineVision>& lanelines_tracked) override;

 private:
  void PreProcess(const Eigen::Matrix4d& pose, const double& timestamp, double& dt);
  void Predict(const double& dt);
  void Match(const std::vector<LaneLineVision>& lanelines_detected);
  void Update(const std::vector<LaneLineVision>& lanelines_detected);
  void PostProcess();
  void Output(std::vector<LaneLineVision>& lanelines_tracked);

  void ClearInvalidTracklets();
  float LaneSimilarity(const LaneTrackletPtr& tracklet, const LaneLineVision& lanes_detected);

 private:
  double last_timestamp_ = 0.0;  // TODO CJS 
  std::unique_ptr<IDPool> id_pool_ptr_;
  std::vector<LaneTrackletPtr> tracklets_;
  LaneQualityEvaluatorPtr lane_quality_evaluator_ptr_;
  
  // basic super-parameters
  std::string camera_name_;
  float iou_thresh_ = 0.5;          ///< iou_thresh_
  int num_anchor_ = 9;                ///< 计算iou时，anchor的数量
  size_t max_id_ = 254;               ///< 跟踪序列ID池的最大值
  size_t min_hits_ = 5;               ///< 最小观测次数
  uint min_world_pts_num_thresh_ = 10;  ///< 车道线最小点数阈值，小于则无效
  float max_start_dist_thresh_ = 15;    ///< 车道线起始点距离阈值，大于则无效
  // tracklet basic-params
  std::shared_ptr<LaneTrackLetInitParam> lane_tracklet_init_param_ptr_;
  //   bool lane_parallelization_ = true;    ///< 是否进行车道线平行化
  Eigen::Matrix4d cur_pose_;  ///< 当前帧自车位姿

  // matching
  std::shared_ptr<ld_post::HungarianAlgorithm> matcher_ptr_;
  std::vector<int> associate_tracklet_ids_;  ///< 关联的跟踪序列在tracklets_中的索引
  std::vector<int> associate_det_ids_;       ///< 关联的检测序列在LDFrame.lanes_detected中的索引
};

} // namespace ld_pos

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END