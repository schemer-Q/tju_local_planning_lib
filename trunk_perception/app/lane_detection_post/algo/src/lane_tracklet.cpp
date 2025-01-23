#include "trunk_perception/app/lane_detection_post/algo/lane_tracklet.h"
#include <numeric>
#include "trunk_perception/app/lane_detection/algo/MathUtil.h"
#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/types/lane.h"
#include "trunk_perception/tools/log/t_log.h"

#include <numeric>

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_post {

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

LaneTracklet::~LaneTracklet() = default;

void LaneTracklet::InitParam(const std::shared_ptr<LaneTrackLetInitParam>& param_ptr){
  occupied_grid_x_start_ = param_ptr->occupied_grid_x_start;
  occupied_grid_x_middle_ = param_ptr->occupied_grid_x_middle;
  occupied_grid_x_end_ = param_ptr->occupied_grid_x_end;
  occupied_x_step_dense_ = param_ptr->occupied_x_step_dense;
  occupied_x_step_sparse_ = param_ptr->occupied_x_step_sparse;
  queue_size_ = param_ptr->queue_size;
  far_x_limit_ = param_ptr->far_x_limit;
  near_x_limit_ = param_ptr->near_x_limit;
  y_limit_ = param_ptr->y_limit;
  fit_pt_num_limit_ = param_ptr->fit_pt_num_limit;
  max_fusion_pt_age_ = param_ptr->max_fusion_pt_age;
  // output criteria
  output_min_quality_thresh_ = param_ptr->output_min_quality_thresh;
  output_min_hits_thresh_ = param_ptr->output_min_hits_thresh;
  output_min_age_thresh_ = param_ptr->output_min_age_thresh;
  // del criteria
  max_lost_age_thresh_ = param_ptr->max_lost_age_thresh;
  min_fusion_pt_num_thresh_ = param_ptr->min_fusion_pt_num_thresh;
}

void LaneTracklet::Predict(const Eigen::Matrix4d& cur_pose) {
  TDEBUG << "LaneTracklet::Predict() start: " << tracklet_id_;
  TDEBUG << "hits_: " << hits_ << " age_: " << age_ << " lost_age_: " << lost_age_;

  predict_with_history_ = false;
  age_++;
  lost_age_++;  // update时，会置零

  bev_anchor_prediction_ = bev_anchor_kf_.predict();
  coeff_pred_ = coeff_kf_.predict();

  // 计算latest_pose到current_pose的变换矩阵
  Eigen::Matrix4d latest_to_cur_pose = cur_pose.inverse() * latest_pose_;
  // TDEBUG << "latest_to_cur_pose: \n" << latest_to_cur_pose;

  // 通过预测anchor位置并重新拟合，将车道线方程预测到当前帧
  PredictFusionPts(latest_to_cur_pose);
  if (predict_with_history_) {
    latest_pose_ = cur_pose;
  }
  TDEBUG << "LaneTracklet::Predict() end";
}

void LaneTracklet::InitFusionPtGridOccupancy() {
  occupied_x_grids_sparse_.resize(int((occupied_grid_x_end_ - occupied_grid_x_middle_) / occupied_x_step_sparse_) + 1);
  occupied_x_grids_dense_.resize(int((occupied_grid_x_middle_ - occupied_grid_x_start_) / occupied_x_step_dense_) + 1);
  std::fill(occupied_x_grids_sparse_.begin(), occupied_x_grids_sparse_.end(), 0);
  std::fill(occupied_x_grids_dense_.begin(), occupied_x_grids_dense_.end(), 0);
}

void LaneTracklet::InitFusionPts() {
  const auto& bev_pts = latest_lane_.OriginPointsWorld;
  for (const auto& pt : bev_pts) {
    fusion_pts_.emplace_back(0, pt);
  }
  // 要求OriginPointsWorld已经按照纵向距离由近至远排序
  near_x_ = bev_pts.front().x;
  far_x_ = bev_pts.back().x;
}

void LaneTracklet::InitKalmanFilter() {
  bev_anchor_kf_.init(N_ANCHOR_POINTS, N_ANCHOR_POINTS, 0);
  bev_anchor_measurement_ = cv::Mat::zeros(N_ANCHOR_POINTS, 1, CV_32F);
  // clang-format off
  bev_anchor_kf_.transitionMatrix = (cv::Mat_<float>(N_ANCHOR_POINTS, N_ANCHOR_POINTS) << 
    1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1);
  // clang-format on
  setIdentity(bev_anchor_kf_.measurementMatrix);
  setIdentity(bev_anchor_kf_.processNoiseCov, cv::Scalar::all(1e-2));
  setIdentity(bev_anchor_kf_.measurementNoiseCov, cv::Scalar::all(1e-1));
  setIdentity(bev_anchor_kf_.errorCovPost, cv::Scalar::all(1));

  // init anchor state(每个anchor_x对应的y方向距离)
  for (int i = 0; i < N_ANCHOR_POINTS; ++i) {
    bev_anchor_kf_.statePost.at<float>(i, 0) = ld_algo::CalculateXValue(latest_lane_, x_anchors_[i]);
  }

  // coeff filter
  coeff_kf_ = cv::KalmanFilter(4, 4, 0);
  coeff_measurement_ = cv::Mat::zeros(4, 1, CV_32F);
  // clang-format off
  coeff_kf_.transitionMatrix = (cv::Mat_<float>(4, 4) << 
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1);
  // clang-format on
  setIdentity(coeff_kf_.measurementMatrix);
  setIdentity(coeff_kf_.processNoiseCov, cv::Scalar::all(1e-2));
  setIdentity(coeff_kf_.measurementNoiseCov, cv::Scalar::all(1e-1));
  setIdentity(coeff_kf_.errorCovPost, cv::Scalar::all(1));

  // init coeff state
  coeff_kf_.statePost.at<float>(0, 0) = latest_lane_.a0;
  coeff_kf_.statePost.at<float>(1, 0) = latest_lane_.a1;
  coeff_kf_.statePost.at<float>(2, 0) = latest_lane_.a2;
  coeff_kf_.statePost.at<float>(3, 0) = latest_lane_.a3;
}

void LaneTracklet::InitAnchors() {
  x_anchors_.clear();
  x_anchors_.resize(N_ANCHOR_POINTS);
  // 从bev x方向最近的点开始，取N_ANCHOR_POINTS个anchor，步长为anchor_step
  float anchor_step = (far_x_ - near_x_) / N_ANCHOR_POINTS;
  for (int i = 0; i < N_ANCHOR_POINTS; ++i) {
    x_anchors_.push_back(near_x_ + i * anchor_step);
  }

  a0_queue_.push_back(latest_lane_.a0);
}

void LaneTracklet::InitCameraProjection() {
  camera_projection_ = GET_CAMERA_PROJECTION(camera_name_);
  if (!camera_projection_) {
    TWARNING << "LaneTracklet::InitCameraProjection failed, camera_projection_ is nullptr: "
             << ErrorCode::UNINITIALIZED;
  }
  camera_undistort_ = GET_CAMERA_UNDISTORT(camera_name_);
  if (!camera_undistort_) {
    TWARNING << "LaneTracklet::InitCameraProjection failed, camera_undistort_ is nullptr: " << ErrorCode::UNINITIALIZED;
  }
}

void LaneTracklet::PredictFusionPts(const Eigen::Matrix4d& latset_to_cur_pose) {
  // clang-format off
  fusion_pts_.erase(
    std::remove_if(fusion_pts_.begin(), fusion_pts_.end(),
      [this, &latset_to_cur_pose](auto& pt) {
        pt.age += 1;
        Eigen::Vector4d last_loc_vec(pt.loc.x, pt.loc.y, 0, 1);
        Eigen::Vector4d curr_loc_vec = latset_to_cur_pose * last_loc_vec;
        
        if (CheckTransformPt(last_loc_vec, curr_loc_vec)) {
          pt.loc.x = float(curr_loc_vec[0]);
          pt.loc.y = float(curr_loc_vec[1]);
          return false;  // 保留此点
        }
        return true;  // 移除此点
      }),
    fusion_pts_.end()
  );
  // clang-format on

  refit_pts_ = GenerateRefitPts();

  // pt num限制
  if (refit_pts_.size() < fit_pt_num_limit_) {
    TWARNING << "LaneTracklet::PredictFusionPts " << tracklet_id_ << " refit failed! valid pt num " << refit_pts_.size()
             << " < " << fit_pt_num_limit_ << "!!!";
    return;
  }

  if (!RefitLaneCoeffs()) {
    TWARNING << "LaneTracklet::PredictFusionPts refit failed!";
    return;
  }

  predict_with_history_ = true;
}

bool LaneTracklet::CheckTransformPt(const Eigen::Vector4d& last_loc_vec, const Eigen::Vector4d& curr_loc_vec) {
  // 纵向距离限制
  if (curr_loc_vec[0] > far_x_limit_ || curr_loc_vec[0] < near_x_limit_) {
    return false;
  }

  // 横向距离限制
  if (fabs(curr_loc_vec[1]) > y_limit_) {
    return false;
  }

  return true;
}

std::vector<cv::Point3f> LaneTracklet::GenerateRefitPts() {
  // 使用 std::sort 替代 std::stable_sort，因为我们不需要保持相等元素的相对顺序
  std::sort(fusion_pts_.begin(), fusion_pts_.end(),
            [](const FusionPt& t1, const FusionPt& t2) { return t1.loc.x < t2.loc.x; });

  std::vector<cv::Point3f> refit_pts;
  refit_pts.reserve(fusion_pts_.size());  // 预分配内存以提高性能

  const float min_distance = 2.0f;
  float last_pt_x = -std::numeric_limits<float>::max();  // 使用最小浮点数作为初始值

  for (const auto& fusion_pt : fusion_pts_) {
    if (fusion_pt.loc.x - last_pt_x > min_distance) {
      refit_pts.emplace_back(fusion_pt.loc.x, fusion_pt.loc.y, 0.0f);
      last_pt_x = fusion_pt.loc.x;
    }
  }

  return refit_pts;
}

bool LaneTracklet::RefitLaneCoeffs() {
  cv::Mat line_params;
  int degree = (poly_num_ == 3) ? 3 : 1;

  if (poly_num_ != 3 && poly_num_ != 1) {
    TWARNING << "LaneTracklet::RefitLaneCoeffs poly_num_ = " << poly_num_ << "! Using 1.";
  }

  if (!ld_algo::PolyFitting(refit_pts_, degree, line_params)) {
    return false;
  }

  latest_lane_.a0 = static_cast<float>(line_params.at<double>(0, 0));
  latest_lane_.a1 = static_cast<float>(line_params.at<double>(1, 0));
  latest_lane_.a2 = (degree == 3) ? static_cast<float>(line_params.at<double>(2, 0)) : 0.f;
  latest_lane_.a3 = (degree == 3) ? static_cast<float>(line_params.at<double>(3, 0)) : 0.f;

  return true;
}

LaneLineVision LaneTracklet::GetLaneWithFusedPts() {
  LaneLineVision lane = latest_lane_;

  // add fusion pts into lane obj
  lane.OriginPointsWorld.clear();
  for (const auto& pt : fusion_pts_) {
    lane.OriginPointsWorld.emplace_back(pt.loc);
  }
  std::stable_sort(lane.OriginPointsWorld.begin(), lane.OriginPointsWorld.end(),
                   [](const cv::Point3f& t1, const cv::Point3f& t2) { return t1.x < t2.x; });

  // update origin image pts using fusion pts sparsified result
  lane.OriginPointsImg.clear();
  if (camera_undistort_ && camera_projection_) {
    for (int i = lane.OriginPointsWorld.size() - 1; i >= 0; i -= 5) {
      auto& pt = lane.OriginPointsWorld[i];
      if (pt.x < 10) {
        continue;
      }
      auto img_pt = camera_projection_->WorldToImg(pt);
      // 可视化图像为原图，对img_pt添加畸变
      img_pt = camera_undistort_->DistortPoint(img_pt);
      if (!camera_projection_->IsPointInView(img_pt)) {
        continue;
      }
      lane.OriginPointsImg.emplace_back(img_pt);
    }
    std::stable_sort(lane.OriginPointsImg.begin(), lane.OriginPointsImg.end(),
                     [](const cv::Point2f& t1, const cv::Point2f& t2) { return t1.y < t2.y; });
  } else {
    TWARNING << "LaneTracklet::GetLaneWithFusedPts OriginPointsImg get failed for camera_projection_ is nullptr: "
             << ErrorCode::UNINITIALIZED;
    InitCameraProjection();
  }

  return lane;
}

void LaneTracklet::Update(const LaneLineVision& det_lane) {
  lost_age_ = 0;
  hits_++;

  // 需要延续的属性
  cv::Scalar show_color = latest_lane_.lane_line_show_color;
  LaneLinePositionIndex lane_index = latest_lane_.lane_line_position;
  // update tracked obj
  latest_lane_ = det_lane;
  latest_lane_.age = age_;
  latest_lane_.track_id = tracklet_id_;
  latest_lane_.lane_line_show_color = show_color;
  latest_lane_.lane_line_position = lane_index;

  // 融合bev点
  if (predict_with_history_) {
    UpdateFusionPts();
  }

  int update_bev_state = UpdateBevState();
  if (update_bev_state != 0) {
    // 更新异常的目标将会被丢弃
    // lost_age_ = max_lost_age_ + 1;
    // 保留更新异常的目标，但不发布
    update_valid_ = false;
  } else {
    update_valid_ = true;
    a0_queue_.push_back(latest_lane_.a0);
    if (a0_queue_.size() > queue_size_) {
      a0_queue_.pop_front();
    }
    float a0_sum = float(std::accumulate(std::begin(a0_queue_), std::end(a0_queue_), 0.0));
    float a0_mean = a0_sum / float(a0_queue_.size());
    float a0_cov = 0.0;
    for (float a0 : a0_queue_) {
      a0_cov += float(pow(a0 - a0_mean, 2));
    }
    latest_lane_.a0_cov = a0_cov;

    // a70 cov
    float a70 = ld_algo::CalculateXValue(latest_lane_, 70);
    a70_queue_.push_back(a70);
    if (a70_queue_.size() > queue_size_) {
      a70_queue_.pop_front();
    }
    float a70_sum = float(std::accumulate(std::begin(a70_queue_), std::end(a70_queue_), 0.0));
    float a70_mean = a70_sum / float(a70_queue_.size());
    float a70_cov = 0.0;
    for (float a70 : a70_queue_) {
      a70_cov += float(pow(a70 - a70_mean, 2));
    }
    latest_lane_.a70_cov = a70_cov;
  }

  latest_lane_.lane_line_type = LaneLineType(type_filter_.Update(latest_lane_.lane_line_type));
  
  // update lane_conf
  float quality_score = quality_estimator_->EvaluateLaneQuality(latest_lane_);
  latest_lane_.lane_conf = quality_score;
}

void LaneTracklet::UpdateFusionPts() {
  // re-init occupied_x_grids
  std::fill(occupied_x_grids_sparse_.begin(), occupied_x_grids_sparse_.end(), 0);
  std::fill(occupied_x_grids_dense_.begin(), occupied_x_grids_dense_.end(), 0);
  // 倒序索引遍历并删除无效点，用更新的点代替旧点
  for (int i = fusion_pts_.size() - 1; i >= 0; --i) {
    if (!CheckFusionPtValid(fusion_pts_[i])) {
      fusion_pts_.erase(fusion_pts_.begin() + i);
    }
  }

  // 加入当前帧的检测点
  for (const auto& detect_pt : latest_lane_.OriginPointsWorld) {
    fusion_pts_.emplace_back(0, detect_pt);
    // 当前帧的检出点全部加入refit_pts_
    refit_pts_.emplace_back(detect_pt);
  }
  // tracked_obj_.OriginPointsWorld = refit_pts_;

  // 使用融合后的点重新拟合
  RefitLaneCoeffs();
}

bool LaneTracklet::CheckFusionPtValid(const FusionPt& pt) {
  // age filter, incase out of memory when keep stationary
  if (pt.age > max_fusion_pt_age_) {
    return false;
  }

  // points sparsifier, using the occupied grid
  // only check in grid pts using the occupied grid, incase of delting pt which is out of grid
  bool in_grid_check = pt.loc.x >= occupied_grid_x_start_ && pt.loc.x <= occupied_grid_x_end_;
  bool in_dense_grid = false, in_sparse_grid = false;
  int occ_x_idx = 0;

  // get index
  if (in_grid_check) {
    if (pt.loc.x < occupied_grid_x_middle_) {  // dense
      in_dense_grid = true;                    // in_dense_grid
      occ_x_idx = int((pt.loc.x - occupied_grid_x_start_) / occupied_x_step_dense_);
      if (occupied_x_grids_dense_[occ_x_idx] == 1) {
        return false;
      }
    } else {
      in_sparse_grid = true;
      occ_x_idx = int((pt.loc.x - occupied_grid_x_middle_) / occupied_x_step_sparse_);
      if (occupied_x_grids_sparse_[occ_x_idx] == 1) {
        return false;
      }
    }
  }

  float offset = std::sqrt(std::pow((pt.loc.x - pt.init_loc.x), 2) + std::pow((pt.loc.y - pt.init_loc.y), 2));
  float threshold = 0.0f;
  float init_x = pt.init_loc.x;
  float abs_init_y = std::fabs(pt.init_loc.y);

  if (init_x > 30.0f) {  // 远处的点 (40+), 推理3帧
    threshold = 5.0f;
  } else if (init_x > 15.0f) {  // 中距离点(10~40)
    if (abs_init_y < 5.0f) {    // 横向距离小于5米的点，推理5帧
      threshold = 25.0f;
    } else {  // 横向距离不小于5米的点，推理10帧
      threshold = 80.0f;
    }
  } else {  // 近距离点，推理15帧
    threshold = 8.0f;
  }

  if (offset < threshold) {  // valid
    if (in_grid_check) {
      if (in_dense_grid) {
        occupied_x_grids_dense_[occ_x_idx] = 1;
      } else if (in_sparse_grid) {
        occupied_x_grids_sparse_[occ_x_idx] = 1;
      }
    }
    return true;
  }
  return false;
}

int LaneTracklet::UpdateBevState() {
  // anchor y
  for (int i = 0; i < N_ANCHOR_POINTS; ++i) {
    float anchor_y = ld_algo::CalculateXValue(latest_lane_, x_anchors_[i]);
    bev_anchor_measurement_.at<float>(i, 0) = anchor_y;
  }
  // correct
  bev_anchor_kf_.correct(bev_anchor_measurement_);

  if (!predict_with_history_) {
    // 未使用历史帧进行推理，则使用预测的y coords重新拟合多项式
    refit_pts_.clear();
    for (int i = 0; i < N_ANCHOR_POINTS; ++i) {
      float predict_y = bev_anchor_kf_.statePost.at<float>(i, 0);
      refit_pts_.emplace_back(x_anchors_[i], predict_y, 0);
    }
    RefitLaneCoeffs();
  }

  /*平滑拟合系数*/
  coeff_measurement_.at<float>(0, 0) = latest_lane_.a0;
  coeff_measurement_.at<float>(1, 0) = latest_lane_.a1;
  coeff_measurement_.at<float>(2, 0) = latest_lane_.a2;
  coeff_measurement_.at<float>(3, 0) = latest_lane_.a3;

  // correct
  coeff_kf_.correct(coeff_measurement_);

  latest_lane_.a0 = coeff_kf_.statePost.at<float>(0, 0);
  latest_lane_.a1 = coeff_kf_.statePost.at<float>(1, 0);
  latest_lane_.a2 = coeff_kf_.statePost.at<float>(2, 0);
  latest_lane_.a3 = coeff_kf_.statePost.at<float>(3, 0);
  return 0;
}

bool LaneTracklet::IsLost() {
  // lost_age thresholding
  if (lost_age_ > max_lost_age_thresh_) {
    TDEBUG<< "[IsLost ] delete lost_age_:" << lost_age_ << " max_lost_age_thresh_:" << max_lost_age_thresh_;
    return true;
  }

  // TODO: 这里需要参数化
  //  pts过少
  if (fusion_pts_.size() < min_fusion_pt_num_thresh_) {
    TDEBUG<< "[IsLost ] delete fusion_pts_.size(): " << fusion_pts_.size() << min_fusion_pt_num_thresh_;
    return true;
  }

  // 已经跑到车辆后方  TODO CJS 
  for (const auto& fusion_pt : fusion_pts_) {
    if (fusion_pt.loc.x > 5) {
      return false;
    }
  }
  return true;
}


bool LaneTracklet::IsMature() const{

  bool res = bev_anchor_kf_.errorCovPre.at<float>(0, 0) < 0.25;  // TODO CJS 
  // tracked param good enough
  bool param_matrue = age_ > 5 && 
                      hits_ >= output_min_hits_thresh_ && 
                      update_valid_;
  res = res && param_matrue;

  // quality high enough
  bool quality_mature = latest_lane_.lane_conf> output_min_quality_thresh_;
  res = res && quality_mature;
  TDEBUG<<"[ISMATURE] "<< "id: "<< tracklet_id_ << " is_hold_on_: " << is_hold_on_ << " lost_age_: " 
        << lost_age_ << " age_:" << age_ << " hits_: " << hits_ << " update_valid_:" << update_valid_ << " param_matrue: " << param_matrue
        << "error_cov_pre: " << bev_anchor_kf_.errorCovPre.at<float>(0, 0) << " lane_conf: " << latest_lane_.lane_conf << " res: "<< res;
  return res;
}
}  // namespace ld_post

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
