
#include "trunk_perception/app/lane_detection_post/algo/tracker/lane_point_tracker.h"
#include "trunk_perception/app/lane_detection/algo/MathUtil.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_post {
  
int LanePointTracker::Init(const YAML::Node& config){
  try{
    // basic param 
    camera_name_ = config["Basic"]["CameraName"].as<std::string>();
    num_anchor_ = config["Basic"]["NumAnchor"].as<int>();
    iou_thresh_ = config["Basic"]["IouThres"].as<float>();
    max_id_ = config["Basic"]["MaxId"].as<int>();
    min_hits_ = config["Basic"]["MinHits"].as<int>();
    min_world_pts_num_thresh_ = config["Basic"]["MinWorldPtsNumThresh"].as<int>();
    max_start_dist_thresh_ = config["Basic"]["MaxStartDistThresh"].as<float>();
    
    // tracklet basic param
    int occ_grid_x_start = config["TrackletBasic"]["OccupiedGridXStart"].as<int>();
    int occ_grid_x_middle = config["TrackletBasic"]["OccupiedGridXMiddle"].as<int>();
    int occ_grid_x_end = config["TrackletBasic"]["OccupiedGridXEnd"].as<int>();
    float occ_x_step_dense = config["TrackletBasic"]["OccupiedXStepDense"].as<float>();
    float occ_x_step_sparse = config["TrackletBasic"]["OccupiedXStepSparse"].as<float>();
    size_t queue_size = config["TrackletBasic"]["QueueSize"].as<size_t>();
    float far_x_limit = config["TrackletBasic"]["FarXLimit"].as<float>();
    float near_x_limit = config["TrackletBasic"]["NearXLimit"].as<float>();
    float y_limit = config["TrackletBasic"]["YLimit"].as<float>();
    size_t fit_pt_num_limit = config["TrackletBasic"]["FitPtNumLimit"].as<size_t>();
    int max_fusion_pt_age = config["TrackletBasic"]["MaxFusionPtAge"].as<int>();
    // output criteria
    float output_min_quality_thresh = config["TrackletBasic"]["OutputQualityMinThresh"].as<float>();
    size_t output_min_hits_thresh = config["TrackletBasic"]["OutputMinHitsThresh"].as<size_t>();
    size_t output_min_age_thresh = config["TrackletBasic"]["OutputMinAgeThresh"].as<size_t>();
    // del criteria
    size_t max_lost_age_thresh = config["TrackletBasic"]["MaxLostAgeThresh"].as<size_t>();
    size_t min_fusion_pt_num_thresh = config["TrackletBasic"]["MinFusionPtsNumThresh"].as<size_t>();

    lane_tracklet_init_param_ptr_ = std::make_shared<ld_post::LaneTrackLetInitParam>(
      occ_grid_x_start, occ_grid_x_middle, occ_grid_x_end,
      occ_x_step_dense, occ_x_step_sparse, queue_size,
      far_x_limit, near_x_limit, y_limit, fit_pt_num_limit, max_fusion_pt_age,
      output_min_quality_thresh, output_min_hits_thresh, output_min_age_thresh,
      max_lost_age_thresh, min_fusion_pt_num_thresh);

    // id pool manager
    id_pool_ptr_ = std::make_unique<IDPool>(max_id_);
    if (id_pool_ptr_ == nullptr) {
      TERROR << "BevLanePostImpl::Init() failed, id_pool_ptr_ is nullptr";
      return ErrorCode::YAML_CONFIG_ERROR;
    }

    // quality
    lane_quality_evaluator_ptr_ = std::make_shared<ld_post::LaneQualityEvaluator>();
    lane_quality_evaluator_ptr_->Init(config["LaneQuality"]);
    if (lane_quality_evaluator_ptr_ == nullptr) {
      TERROR << "BevLanePostImpl::Init() failed, lane_quality_evaluator_ptr_ is nullptr";
      return ErrorCode::YAML_CONFIG_ERROR;
    }

  } catch (const std::exception& e) {
    TFATAL << "LanePointTracker::Init() failed, ";
    return ErrorCode::YAML_CONFIG_ERROR;
  }

  return ErrorCode::SUCCESS;
}

int LanePointTracker::Run(const std::vector<LaneLineVision>& lanelines_detected, 
                          const Eigen::Matrix4d& pose, 
                          const double timestamp,
                          std::vector<LaneLineVision>& lanelines_tracked) {
  double dt = 0.0;
  PreProcess(pose, timestamp, dt);
  Predict(dt);
  Match(lanelines_detected);
  Update(lanelines_detected);
  PostProcess();

  Output(lanelines_tracked);

  return ErrorCode::SUCCESS;
}

void LanePointTracker::PreProcess(const Eigen::Matrix4d& pose, const double& timestamp, double& dt){
  cur_pose_ = pose;
  dt = timestamp - last_timestamp_;
  last_timestamp_ = timestamp;
}

void LanePointTracker::Predict(const double& dt){
  // TODO CJS 后续需要修改为按照dt进行预测
  TDEBUG << "LanePointTracker::predict() start";
  TDEBUG << "tracklets_ num: " << tracklets_.size();
  for (auto& tracklet : tracklets_) {
    tracklet->Predict(cur_pose_);
  }
  TDEBUG << "BevLanePostImpl::Predict() end";
}

void LanePointTracker::Match(const std::vector<LaneLineVision>& lanelines_detected){
  TDEBUG << "BevLanePostImpl::Associate() start";
  TDEBUG << ">>>>>>>>>>>>> tracklets_ num: " << tracklets_.size();
  TDEBUG << ">>>>>>>>>>>>> lanelines_detected num: " << lanelines_detected.size();

  associate_tracklet_ids_.clear();
  associate_det_ids_.clear();

  size_t tracklet_num = tracklets_.size();
  size_t det_num = lanelines_detected.size();

  if (tracklet_num == 0 || det_num == 0) {
    return;
  }

  std::vector<std::vector<double>> cost_matrix(tracklet_num, std::vector<double>(det_num, 0));

  // compute iou matrix as a distance matrix
  for (size_t i = 0; i < tracklet_num; i++) {
    const auto& tracklet = tracklets_[i];
    // 清除tracker的关联得分
    tracklet->ResetAssociateScore();
    for (size_t j = 0; j < det_num; j++) {
      float similarity = LaneSimilarity(tracklet, lanelines_detected[j]);
      // use 1 - similarity because the hungarian algorithm
      cost_matrix[i][j] = 1 - similarity;
    }
    // std::string match_similarity = std::to_string(tracklet->GetTrackletId()) + " -> [ ";
    // for (size_t j = 0; j < det_num; j++) {
    //   match_similarity = match_similarity+ std::to_string(cost_matrix[i][j])+" ";
    // }
    // match_similarity = match_similarity + "]";
    // TDEBUG<<match_similarity;
  }

  std::vector<int> assignment;  // assignment保存为每个pred_bbox分配的检测目标的索引
  matcher_ptr_->Solve(cost_matrix, assignment);

  for (size_t i = 0; i < assignment.size(); ++i) {
    int det_idx = assignment[i];
    if (det_idx == -1) continue;
    if (1 - cost_matrix[i][assignment[i]] >= iou_thresh_) {
      associate_det_ids_.push_back(det_idx);
      associate_tracklet_ids_.push_back(i);
    }
  }

  TDEBUG << "BevLanePostImpl::Associate() end";
}

void LanePointTracker::Update(const std::vector<LaneLineVision>& lanelines_detected){
  // TODO CJS 
  for (size_t i = 0; i < associate_det_ids_.size(); ++i) {
    int tracklet_index = associate_tracklet_ids_[i];
    int detect_index = associate_det_ids_[i];
    auto& tracklet = tracklets_[tracklet_index];
    auto& det_obj = lanelines_detected[detect_index];
    TDEBUG<<"[TRK Update] match, trk: "<< tracklet->GetTrackletId() << " to det:" << detect_index;
    // 更新目标
    tracklet->Update(det_obj);
  }

  // create new targets
  for (size_t j = 0; j < lanelines_detected.size(); ++j) {
    if (std::find(associate_det_ids_.begin(), associate_det_ids_.end(), j) != associate_det_ids_.end()) {
      continue;
    }
    int new_id = id_pool_ptr_->GetNewId();
    if (new_id < 0) {
      TERROR << "BevLanePostImpl::Update() id pool running out!!!";
      continue;
    }
    // 未被关联的检测目标，为其创建新的target
    auto new_tracklet = std::make_shared<LaneTracklet>(new_id, lanelines_detected[j], cur_pose_, camera_name_, 
                                                       lane_quality_evaluator_ptr_, lane_tracklet_init_param_ptr_);
    tracklets_.push_back(new_tracklet);
    TDEBUG << "[====== create new target===]: " << "new id: " << new_id << ", detect index: " << j;
  }

}

void LanePointTracker::Output(std::vector<LaneLineVision>& lanelines_tracked){
  lanelines_tracked.clear();
  lanelines_tracked.reserve(tracklets_.size());
  TDEBUG <<" [Output debug] tracklets_ size: " << tracklets_.size();
  for (const auto& tracklet : tracklets_) {
    if (tracklet->IsMature()) {
      auto res = tracklet->GetLaneWithFusedPts();
      if (res.OriginPointsWorld.size() < min_world_pts_num_thresh_ ||
          res.OriginPointsWorld.front().x > max_start_dist_thresh_) {
        continue;
      }
      lanelines_tracked.push_back(res);
    }
  }

  // TODO CJS 平行处理的代码实现
  // if (lane_parallelization_ && lanelines_tracked.size() >= 2) {
  //   std::vector<int> valid_lane_idxs;
  //   // get line pts
  //   std::vector<std::vector<std::pair<float, float>>> points;
  //   for (size_t idx = 0; idx < lanelines_tracked.size(); idx++) {
  //     // todo cjs
  //     const auto& obj = lanelines_tracked[idx];
  //     std::vector<std::pair<float, float>> lane_pts;
  //     int near_x = 0;
  //     int far_x = 0;
  //     if (fabs(obj.a0) < 4) {
  //       near_x = egolane_parallel_fit_near_x_;
  //       far_x = egolane_parallel_fit_far_x_;
  //     } else {
  //       near_x = 25;
  //       far_x = 35;
  //     }
  //     for (size_t i = 0; i < obj.OriginPointsWorld.size(); ++i) {
  //       if (obj.OriginPointsWorld[i].x >= near_x && obj.OriginPointsWorld[i].x <= far_x) {
  //         lane_pts.push_back({obj.OriginPointsWorld[i].x, obj.OriginPointsWorld[i].y});
  //       }
  //     }
  //     if (lane_pts.size() > 3) {
  //       valid_lane_idxs.push_back(idx);
  //       points.emplace_back(lane_pts);
  //     } else {
  //       TDEBUG << "JUMP ONE LANE";
  //     }
  //   }

  //   // point spasify
  //   for (auto& point : points) {
  //     point = SparsifyPoints(point);
  //   }

  //   // get parallelized params
  //   std::vector<ld_algo::PolynomialCoefficients> parallel_fit_params = ld_algo::FitParallelLines(points, false, 3);

  //   for (size_t i = 0; i < valid_lane_idxs.size(); i++) {
  //     auto& line = lanelines_tracked[valid_lane_idxs[i]];
  //     auto& param = parallel_fit_params[i];
  //     line.a0 = param.a0;
  //     line.a1 = param.a1;
  //     line.a2 = param.a2;
  //     line.a3 = param.a3;
  //   }
  // }

  TDEBUG << "[Output debug] GenerateTracklets: " << lanelines_tracked.size() << " lanes";
}

void LanePointTracker::PostProcess(){
  ClearInvalidTracklets();
}

void LanePointTracker::ClearInvalidTracklets() {
  auto new_end = std::remove_if(tracklets_.begin(), tracklets_.end(), [this](const auto& tracklet) {
    if (tracklet->IsLost()) {
      id_pool_ptr_->RecycleId(tracklet->GetTrackletId());
      return true;  // 标记为需要删除
    }
    return false;
  });
  tracklets_.erase(new_end, tracklets_.end());
}


float LanePointTracker::LaneSimilarity(const LaneTrackletPtr& tracklet, const LaneLineVision& lanelines_detected) {
  // 1. Calculate x-axis overlap (IOU)
  float det_min_x = lanelines_detected.OriginPointsWorld.front().x;
  float det_max_x = lanelines_detected.OriginPointsWorld.back().x;
  LaneLineVision tracked_lane = tracklet->GetLaneWithFusedPts();
  float trk_min_x = std::max(0.0f, tracked_lane.OriginPointsWorld.front().x);
  float trk_max_x = tracked_lane.OriginPointsWorld.back().x;

  float overlap_start = std::max(det_min_x, trk_min_x);
  float overlap_end = std::min(det_max_x, trk_max_x);
  float overlap_length = overlap_end - overlap_start;

  // No overlap, return 0
  if (overlap_length <= 1e-6) {
    return 0.0f;
  }

  // Calculate IOU on x-axis
  float total_length = std::max(det_max_x, trk_max_x) - std::min(det_min_x, trk_min_x);
  float iou_x = overlap_length / total_length;

  // 2. Calculate y-axis deviation ONLY in the overlapping region
  float y_dis = 0.0f;
  int valid_anchor_count = 0;
  const int num_anchor = 10;  

  // Generate anchors within the overlapping region
  float anchor_step = overlap_length / (num_anchor - 1);
  for (int i = 0; i < num_anchor; ++i) {
    float anchor_x = overlap_start + i * anchor_step;

    // Interpolate y-values for both lanes at anchor_x
    float y_det = ld_algo::CalculateXValue(lanelines_detected, anchor_x);
    float y_trk = ld_algo::CalculateXValue(tracklet->GetLaneFitParam(), anchor_x);

    // Weighted distance (e.g., prioritize near-field with decay)
    float decay = std::max(0.1f, 1 - anchor_x / 60.0f);  // TODO CJS magic number
    y_dis += std::abs(y_trk - y_det) * decay;
    valid_anchor_count++;
  }

  if (valid_anchor_count == 0) return 0.0f;

  // 3. Dynamic threshold based on overlap ratio
  float avg_y_dis = y_dis / valid_anchor_count;

  // 4. Combine IOU and y-axis deviation
  float similarity = iou_x * std::max(0.0f, 1 - avg_y_dis);
  similarity = std::clamp(similarity, 0.0f, 1.0f);

  // Update tracklet score if higher
  if (similarity > tracklet->GetAssociateScore()) {
    tracklet->SetAssociateScore(similarity);
  }

  return similarity;
}


}  // ld_post

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END