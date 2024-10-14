#include "trunk_perception/app/lane_detection_post/bev_lane_post_impl.h"
#include <cstddef>
#include <memory>
#include "trunk_perception/app/lane_detection/algo/MathUtil.h"
#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/lane.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

BevLanePostImpl::BevLanePostImpl() { id_pool_ = std::unique_ptr<ld_post::IDPool>(new ld_post::IDPool(max_id_)); }

BevLanePostImpl::~BevLanePostImpl() = default;

std::uint32_t BevLanePostImpl::Init(const YAML::Node& config) {
  try {
    camera_name_ = config["Name"].as<std::string>();
  } catch (const std::exception& e) {
    TFATAL << "BevLanePostImpl::Init() failed, " << e.what();
    return ErrorCode::YAML_CONFIG_ERROR;
  }
  return ErrorCode::SUCCESS;
}

std::uint32_t BevLanePostImpl::Run(const double& ts) {
  // 获取当前帧数据
  auto ld_frame = GET_LD_FRAME();
  if (ld_frame == nullptr) {
    TWARNING << "BevLanePostImpl::Run() ld_frame is nullptr";
    return ErrorCode::LANE_TRACKER_LD_FRAME_NOT_FOUND;
  }

  // NOTE: 这里跟原SDK有区别
  // 原SDK: 转移矩阵失效时，标记tracklet为lost，会被删除
  // 这里: Odom获取失败，放弃当前帧的跟踪，返回错误码，已有tacklet不会被删除
  std::shared_ptr<OdometryData> odometry;
  if (GET_ODOMETRY_DATA_BY_TIME(ld_frame->timestamp, odometry) != ErrorCode::SUCCESS) {
    return ErrorCode::LANE_TRACKER_ODOMETRY_NOT_FOUND;
  }
  if (odometry == nullptr || odometry->data == nullptr) {
    return ErrorCode::LANE_TRACKER_ODOMETRY_NOT_FOUND;
  }
  cur_pose_ = odometry->data->Matrix();

  Predict();
  Associate(ld_frame->lanes_detected);
  Update(ld_frame->lanes_detected);
  ClearTracklet();
  // 车道线位置选取
  if (lane_position_select_) {
    LanePositionProcess();
    // set the hold on lanelines
    if (egolane_hold_on_mode_) {
      EgoLaneHoldOnProcess();
    }
  }
  GenerateTracklets(ld_frame->lanes_tracked);
  return ErrorCode::SUCCESS;
}

std::any BevLanePostImpl::GetData(const std::string& key) { return std::any(); }

void BevLanePostImpl::EstimatePolyFittingNum() {
  // TODO: 待实现
}

void BevLanePostImpl::Predict() {
  TDEBUG << "BevLanePostImpl::Predict() start";
  TDEBUG << "tracklets_ num: " << tracklets_.size();
  for (auto& tracklet : tracklets_) {
    tracklet->Predict(cur_pose_);
  }
  TDEBUG << "BevLanePostImpl::Predict() end";
}

void BevLanePostImpl::Associate(const std::vector<LaneLineVision>& lanes_detected) {
  TDEBUG << "BevLanePostImpl::Associate() start";
  TDEBUG << ">>>>>>>>>>>>> tracklets_ num: " << tracklets_.size();
  TDEBUG << ">>>>>>>>>>>>> lanes_detected num: " << lanes_detected.size();

  associate_tracklet_ids_.clear();
  associate_det_ids_.clear();

  size_t tracklet_num = tracklets_.size();
  size_t det_num = lanes_detected.size();

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
      float similarity = LaneSimilarity(tracklet, lanes_detected[j]);
      // use 1 - similarity because the hungarian algorithm
      cost_matrix[i][j] = 1 - similarity;
    }
  }

  std::vector<int> assignment;  // assignment保存为每个pred_bbox分配的检测目标的索引
  hungarian_algo_.Solve(cost_matrix, assignment);

  for (size_t i = 0; i < assignment.size(); ++i) {
    int det_idx = assignment[i];
    if (det_idx == -1) continue;
    if (1 - cost_matrix[i][assignment[i]] >= iou_thresh_) {
      associate_det_ids_.push_back(det_idx);
      associate_tracklet_ids_.push_back(i);
    }
  }

  TDEBUG << ">>>>>>>>>> associate_tracklet_ids_: " << associate_tracklet_ids_;
  TDEBUG << ">>>>>>>>>> associate_det_ids_: " << associate_det_ids_;
  TDEBUG << "BevLanePostImpl::Associate() end";
}

float BevLanePostImpl::LaneSimilarity(const LaneTrackletPtr& tracklet, const LaneLineVision& lanes_detected) {
  float iou = 0.;

  // x-axis IOU check
  float det_min_x = lanes_detected.OriginPointsWorld.front().x;
  float det_max_x = lanes_detected.OriginPointsWorld.back().x;
  LaneLineVision tracked_lane = tracklet->GetLaneWithFusedPts();
  float trk_min_x = std::max(0.0f, tracked_lane.OriginPointsWorld.front().x);
  float trk_max_x = tracked_lane.OriginPointsWorld.back().x;

  float min_of_max_xs = std::min(det_max_x, trk_max_x);
  float max_of_min_xs = std::max(det_min_x, trk_min_x);
  float max_x = std::max(det_max_x, trk_max_x);
  float min_x = std::min(det_min_x, trk_min_x);

  // Check if there is an overlap on the x-axis
  if (min_of_max_xs <= max_of_min_xs + 1e-6) {
    return 0.0f;
  }

  // Calculate IOU on the x-axis
  float iou_x = (min_of_max_xs - max_of_min_xs) / (max_x - min_x);
  if (iou_x < 0.2f) {
    return 0.0f;
  }

  // anchor dis_offset
  auto& bev_pts_det = lanes_detected.OriginPointsWorld;
  float anchor_step_det = (bev_pts_det.back().x - bev_pts_det.front().x) / num_anchor_;
  float y_dis = 1e-6;
  for (int i = 0; i < num_anchor_; ++i) {
    float anchor_x_det = bev_pts_det.front().x + i * anchor_step_det;
    float anchor_y_det = ld_algo::CalculateXValue(lanes_detected, anchor_x_det);
    float anchor_y_pred = ld_algo::CalculateXValue(tracklet->GetLaneFitParam(), anchor_x_det);
    float decay = std::max(0.1f, 1 - anchor_x_det / 70.0f);
    y_dis += fabs(anchor_y_pred - anchor_y_det) * decay;
  }
  float avg_y_dis = y_dis / num_anchor_;

  if (avg_y_dis >= 1) {
    return iou;
  }
  iou = 1 - avg_y_dis;

  if (iou > tracklet->GetAssociateScore()) {
    tracklet->SetAssociateScore(iou);
  }

  return iou;
}

void BevLanePostImpl::Update(const std::vector<LaneLineVision>& lanes_detected) {
  // 更新目标状态
  for (size_t i = 0; i < associate_det_ids_.size(); ++i) {
    int tracklet_index = associate_tracklet_ids_[i];
    int detect_index = associate_det_ids_[i];
    auto& tracklet = tracklets_[tracklet_index];
    auto& det_obj = lanes_detected[detect_index];
    // 更新目标
    tracklet->Update(det_obj);
  }

  // create new targets
  for (size_t j = 0; j < lanes_detected.size(); ++j) {
    if (std::find(associate_det_ids_.begin(), associate_det_ids_.end(), j) != associate_det_ids_.end()) {
      continue;
    }
    int new_id = id_pool_->GetNewId();
    if (new_id < 0) {
      TERROR << "BevLanePostImpl::Update() id pool running out!!!";
      continue;
    }
    // 未被关联的检测目标，为其创建新的target
    auto new_tracklet = std::make_shared<ld_post::LaneTracklet>(new_id, lanes_detected[j], cur_pose_, camera_name_);
    tracklets_.push_back(new_tracklet);
    TDEBUG << "create new tracklet " << new_tracklet->GetTrackletId();
  }
}

void BevLanePostImpl::ClearTracklet() {
  dropped_lanes_.clear();

  // 使用反向迭代器来避免在删除元素时的索引问题
  for (auto it = tracklets_.rbegin(); it != tracklets_.rend();) {
    auto& tracklet = *it;
    if (tracklet->IsLost()) {
      dropped_lanes_.push_back(tracklet->GetLaneWithFusedPts());

      switch (tracklet->GetLanePosIndex()) {
        case LaneLinePositionIndex::LEFT_1:
          ResetEgoLeftLane();
          break;
        case LaneLinePositionIndex::RIGHT_1:
          ResetEgoRightLane();
          break;
        case LaneLinePositionIndex::LEFT_2:
          ResetLeftLeftLane();
          break;
        case LaneLinePositionIndex::RIGHT_2:
          ResetRightRightLane();
          break;
        default:
          break;
      }

      id_pool_->RecycleId(tracklet->GetTrackletId());
      it = std::reverse_iterator<decltype(tracklets_)::iterator>(tracklets_.erase((++it).base()));
    } else {
      ++it;
    }
  }
}

void BevLanePostImpl::LanePositionProcess() {
  // 车道线从左至右排序
  std::stable_sort(tracklets_.begin(), tracklets_.end(), [&](const LaneTrackletPtr& t1, const LaneTrackletPtr& t2) {
    return t1->GetFitA0() > t2->GetFitA0();
  });

  // 获取右侧第一根车道线位置
  iter_right_1_ = std::find_if(tracklets_.begin(), tracklets_.end(),
                               [&](const LaneTrackletPtr& tracklet) { return tracklet->GetFitA0() < 0; });

  EgoLaneProcess();

  // 左左 右右 更新
  AdjacentLaneProcess();

  for (const auto& tracklet : tracklets_) {
    if (ego_left_tracked_ && tracklet->GetTrackletId() == ego_left_target_->GetTrackletId()) {
      SetEgoLeftLane(tracklet);
    } else if (ego_right_tracked_ && tracklet->GetTrackletId() == ego_right_target_->GetTrackletId()) {
      SetEgoRightLane(tracklet);
    } else if (left_left_tracked_ && tracklet->GetTrackletId() == left_left_target_->GetTrackletId()) {
      SetLeftLeftLane(tracklet);
    } else if (right_right_tracked_ && tracklet->GetTrackletId() == right_right_target_->GetTrackletId()) {
      SetRightRightLane(tracklet);
    }
  }
}

void BevLanePostImpl::EgoLaneUpdate() {
  // note:跟踪丢失会导致ego lane更新失败，重新选取
  ego_left_tracked_ = ego_left_target_ != nullptr && ego_left_target_->IsTracked();
  ego_right_tracked_ = ego_right_target_ != nullptr && ego_right_target_->IsTracked();

  // ego lane track结果校验, 校验失败则重新初始化ego lane
  if (!ego_left_tracked_ && !ego_right_tracked_) {
    // 两侧ego lane均丢失，重新尝试match ego pair lane
    ResetEgoLeftLane();
    ResetEgoRightLane();
    InitEgoLane();
    return;
  } else if (ego_left_tracked_ && !ego_right_tracked_) {
    // 校验ego left
    if (!CheckEgoLaneLeftQualityLoosely()) {
      ResetEgoLeftLane();
      InitEgoLane();
      return;
    }
  } else if (ego_right_tracked_ && !ego_left_tracked_) {
    // 校验ego right
    if (!CheckEgoLaneRightQualityLoosely()) {
      ResetEgoRightLane();
      InitEgoLane();
      return;
    }
  } else {
    // 校验两侧ego lane
    if (!CheckEgoLaneUnionQualityLoosely()) {
      ResetEgoLeftLane();
      ResetEgoRightLane();
      InitEgoLane();
      return;
    }
  }

  // 有任意一侧ego lane更新成功，则尝试匹配另一侧
  if (ego_left_tracked_ && !ego_right_tracked_) {
    TDEBUG << "Only left ego lane updated, Now search right side...";
    // 仅ego right丢失，尝试重新匹配ego right
    ResetEgoRightLane();  // 先做清理
    if (TryingMatchEgoPairLane(LaneLinePositionIndex::RIGHT_1)) {
      TDEBUG << "right ego lane reinit success~";
    } else {
      TWARNING << "BevLanePostImpl::EgoLaneUpdate: right ego lane reinit failed!";
    }
  } else if (!ego_left_tracked_ && ego_right_tracked_) {
    TDEBUG << "Only right ego lane updated, Now search left side...";
    // 仅ego left丢失，尝试重新匹配ego left
    ResetEgoLeftLane();  // 先做清理
    if (TryingMatchEgoPairLane(LaneLinePositionIndex::LEFT_1)) {
      TDEBUG << "left ego lane reinit success~";
    } else {
      TWARNING << "BevLanePostImpl::EgoLaneUpdate: left ego lane reinit failed!";
    }
  }

  // 两侧ego lane均更新成功，做变道判断
  if (ego_left_tracked_ && ego_right_tracked_ && IsChangeLane()) {
    // 变道后重新初始化ego lane
    ResetEgoLeftLane();
    ResetEgoRightLane();
    InitEgoLane();
  }
}

bool BevLanePostImpl::InitEgoLane() {
  TDEBUG << "BevLanePostImpl::InitEgoLane trying to init ego lane...";
  if (tracklets_.size() < 2) {
    TWARNING << "BevLanePostImpl::InitEgoLane failed, target lanes < 2";
    return false;
  }

  // if no left lane
  if (iter_right_1_ == tracklets_.begin()) {
    TWARNING << "BevLanePostImpl::InitEgoLane failed: no left side lane";
    return false;
  } else if (iter_right_1_ == tracklets_.end()) {
    TWARNING << "BevLanePostImpl::InitEgoLane failed: no right side lane";
    return false;
  }

  for (auto iter_left = iter_right_1_ - 1; iter_left >= tracklets_.begin(); iter_left--) {
    ///! 最多向单侧搜寻三根车道线
    if (iter_left - iter_right_1_ < -3) {
      break;
    }
    for (auto iter_right = iter_right_1_; iter_right < tracklets_.end(); iter_right++) {
      // too far away
      if (iter_right - iter_right_1_ > 3) {
        continue;
      }
      // check the matchness of the selected lane pair
      if (EgoPairLaneMatch(*iter_left, *iter_right)) {
        SetEgoLeftLane(*iter_left);
        SetEgoRightLane(*iter_right);
        return true;
      }
    }
  }
  return false;
}

bool BevLanePostImpl::EgoPairLaneMatch(const LaneTrackletPtr& lane_left, const LaneTrackletPtr& lane_right) {
  /// TODO: cjs code_test 统计多个位置的横向坐标对比，防止曲率过大的误检车道线干扰
  // 2.2 check world loc relative and absolute position
  // Note : 坐标系是左正右负
  // TODO: 需要参数化
  if (lane_left->GetFitA0() > 5.5 || lane_left->GetFitA0() < -1.8) {
    TWARNING << "BevLanePostImpl::EgoPairLaneMatch: left lane too left {0:.2f}", lane_left->GetFitA0();
    return false;
  }
  if (lane_right->GetFitA0() < -5.5 || lane_right->GetFitA0() > 1.8) {
    TWARNING << "BevLanePostImpl::EgoPairLaneMatch: right lane too right {0:.2f}", lane_right->GetFitA0();
    return false;
  }
  float lane_width = std::fabs(lane_right->GetFitA0() - lane_left->GetFitA0());
  if (lane_width < 2 || lane_width > 5) {
    TWARNING << "BevLanePostImpl::EgoPairLaneMatch: lane width not right " << lane_width;
    return false;
  }
  TDEBUG << "BevLanePostImpl::EgoPairLaneMatch: match ego pair successful {0:.2f} {1:.2f}", lane_left->GetFitA0(),
      lane_right->GetFitA0();
  return true;
}

int BevLanePostImpl::CheckEgoLaneLeftQualityLoosely() {
  float left_a0 = ego_left_target_->GetFitA0();
  // intercept value
  if (left_a0 < 0 || left_a0 > 3.75) {
    TWARNING << "BevLanePostImpl::CheckEgoLaneLeftQualityLoosely: left ego lane check invalid a0: " << left_a0;
    return 0;
  }
  return 3;
}

int BevLanePostImpl::CheckEgoLaneRightQualityLoosely() {
  float right_a0 = ego_right_target_->GetFitA0();
  // intercept value
  if (right_a0 > 0 || right_a0 < -3.75) {
    TWARNING << "BevLanePostImpl::CheckEgoLaneRightQualityLoosely: right ego lane check invalid a0: " << right_a0;
    return 0;
  }
  return 3;
}

int BevLanePostImpl::CheckEgoLaneUnionQualityLoosely() {
  float left_a0 = ego_left_target_->GetFitA0();
  float right_a0 = ego_right_target_->GetFitA0();
  if (left_a0 > 5.5 || left_a0 < -1.8) {
    TWARNING << "BevLanePostImpl::CheckEgoLaneUnionQualityLoosely: left ego lane check invalid a0: " << left_a0;
    return 0;
  }
  if (right_a0 < -5.5 || right_a0 > 1.8) {
    TWARNING << "BevLanePostImpl::CheckEgoLaneUnionQualityLoosely: right ego lane check invalid a0: " << right_a0;
    return 0;
  }
  float lane_width = std::fabs(right_a0 - left_a0);
  if (lane_width < 2.5 || lane_width > 5.2) {
    TWARNING << "BevLanePostImpl::CheckEgoLaneUnionQualityLoosely: lane width invalid " << lane_width;
    return 0;
  }
  return 3;
}

bool BevLanePostImpl::TryingMatchEgoPairLane(LaneLinePositionIndex lane_index) {
  LaneTrackletPtr ex_target;
  if (lane_index == LaneLinePositionIndex::LEFT_1) {
    // 存在右侧，搜索左侧
    ex_target = ego_right_target_;
  } else if (lane_index == LaneLinePositionIndex::RIGHT_1) {
    // 存在左侧，搜索右侧
    ex_target = ego_left_target_;
  } else {
    return false;
  }
  if (lane_index == LaneLinePositionIndex::LEFT_1) {
    if (iter_right_1_ == tracklets_.begin()) {
      // 需要match ego left，但没有left lane
      TWARNING << "BevLanePostImpl::TryingMatchEgoPairLane: right search ego left pair failed. no left side lane";
      return false;
    }

    // 尝试匹配左侧ego lane
    const auto& left1_lane_target = *(iter_right_1_ - 1);
    TDEBUG << "ego right " << ex_target->GetFitA0() << " trying match ego left " << left1_lane_target->GetFitA0();
    if (EgoPairLaneMatch(left1_lane_target, ex_target)) {
      SetEgoLeftLane(left1_lane_target);
      return true;
    }
  } else if (lane_index == LaneLinePositionIndex::RIGHT_1) {
    if (iter_right_1_ == tracklets_.end()) {
      // 需要match ego right，但没有right lane
      TWARNING << "BevLanePostImpl::TryingMatchEgoPairLane: left search ego right pair failed. no right side lane";
      return false;
    }

    // 尝试匹配右侧ego lane
    const auto& right1_lane_target = *(iter_right_1_);
    TDEBUG << "ego left " << ex_target->GetFitA0() << " trying match ego right " << right1_lane_target->GetFitA0();
    if (EgoPairLaneMatch(ex_target, right1_lane_target)) {
      SetEgoRightLane(right1_lane_target);
      return true;
    }
  }

  return false;
}

bool BevLanePostImpl::IsChangeLane() {
  if (ego_left_target_->GetFitA0() < 0) {
    TDEBUG << "change lane to left side!";
    return true;
  }
  if (ego_right_target_->GetFitA0() > 0) {
    TDEBUG << "change lane to right side!";
    return true;
  }

  return false;
}

void BevLanePostImpl::EgoLaneCheck() {
  if (ego_left_tracked_ && ego_right_tracked_) {
    TDEBUG << "EgoLane check with both side";
    if (CheckEgoLaneUnionQualityLoosely() < 2) {
      ResetEgoLeftLane();
      ResetEgoRightLane();
      TWARNING << "BevLanePostImpl::EgoLaneCheck: both ego lane check failed!";
    } else {
      TDEBUG << "EgoLane valid " << ego_left_target_->GetTrackletId() << " " << ego_left_target_->GetFitA0() << " "
             << ego_right_target_->GetTrackletId() << " " << ego_right_target_->GetFitA0();
    }
  } else if (ego_left_tracked_ && !ego_right_tracked_) {
    TDEBUG << "EgoLane check with left side";
    if (CheckEgoLaneLeftQualityLoosely() < 2) {
      TWARNING << "BevLanePostImpl::EgoLaneCheck: left ego lane check failed!";
      ResetEgoLeftLane();
    } else {
      TDEBUG << "Ego left valid " << ego_left_target_->GetTrackletId() << " " << ego_left_target_->GetFitA0();
    }
  } else if (!ego_left_tracked_ && ego_right_tracked_) {
    TDEBUG << "EgoLane check with right side";
    if (CheckEgoLaneRightQualityLoosely() < 2) {
      TWARNING << "BevLanePostImpl::EgoLaneCheck: right ego lane check failed!";
      ResetEgoRightLane();
    } else {
      TDEBUG << "Ego right valid " << ego_right_target_->GetTrackletId() << " " << ego_right_target_->GetFitA0();
    }
  } else {
    TWARNING << "EgoLane Check, both side lost, quit!";
    ResetEgoLeftLane();
    ResetEgoRightLane();
  }
}

void BevLanePostImpl::EgoLaneRefit() {
  if (!ego_left_tracked_ || !ego_right_tracked_) {
    return;
  }
  std::vector<cv::Point3f> left_world_pts = ego_left_target_->GetRefitPts();
  std::vector<cv::Point3f> right_world_pts = ego_right_target_->GetRefitPts();
  int left_num = int(left_world_pts.size());
  int total_pt_num = left_num + int(right_world_pts.size());

  // get left lane info
  Eigen::MatrixXf A(total_pt_num, 5);
  Eigen::VectorXf b(total_pt_num);
  for (int i = 0; i < left_num; ++i) {
    const auto& left_pt = left_world_pts[i];
    A(i, 0) = 1.0f;
    A(i, 1) = 0.0f;
    float x = left_pt.x * 0.1f;
    A(i, 2) = x;
    A(i, 3) = x * x;
    A(i, 4) = x * x * x;
    b(i) = left_pt.y;
  }

  // get right lane info
  for (int i = left_num; i < total_pt_num; ++i) {
    const auto& right_pt = right_world_pts[i - left_num];
    A(i, 0) = 0.0f;
    A(i, 1) = 1.0f;
    float x = right_pt.x * 0.1f;
    A(i, 2) = x;
    A(i, 3) = x * x;
    A(i, 4) = x * x * x;
    b(i) = right_pt.y;
  }
  // solve parameters
  Eigen::VectorXf coeffs(5);
  coeffs = A.colPivHouseholderQr().solve(b);
  ego_left_target_->SetFitParam(coeffs[0], coeffs[2] * 0.1f, coeffs[3] * 0.01f, coeffs[4] * 0.001f);
  ego_right_target_->SetFitParam(coeffs[1], coeffs[2] * 0.1f, coeffs[3] * 0.01f, coeffs[4] * 0.001f);
  TDEBUG << "refit ego lane, left: " << coeffs[0] << " right : " << coeffs[1];
}

void BevLanePostImpl::EgoLaneProtect() {
  if (!ego_left_tracked_ || !ego_right_tracked_) {
    return;
  }

  for (const auto& tracklet : tracklets_) {
    if (tracklet->GetLanePosIndex() == LaneLinePositionIndex::RIGHT_1 ||
        tracklet->GetLanePosIndex() == LaneLinePositionIndex::LEFT_1) {
      continue;
    }
    // todo
  }
}

void BevLanePostImpl::AdjacentLaneProcess() {
  left_left_tracked_ = left_left_target_ != nullptr && left_left_target_->IsTracked();
  right_right_tracked_ = right_right_target_ != nullptr && right_right_target_->IsTracked();

  // left left是否依然存在且未被占用
  if (left_left_tracked_) {
    TDEBUG << "trying update left left...";
    if (left_left_target_->GetLanePosIndex() == LaneLinePositionIndex::LEFT_1 ||
        left_left_target_->GetLanePosIndex() == LaneLinePositionIndex::RIGHT_1) {
      TDEBUG << "left left occupied, reset...";
      ResetLeftLeftLane();
    } else {
      auto left_left_a0 = left_left_target_->GetFitA0();
      // 判断left left是否依然合法
      if (CheckLLLinePositive(left_left_target_)) {
        TDEBUG << "left left updated " << left_left_a0;
        SetLeftLeftLane(left_left_target_);
      } else {
        ResetLeftLeftLane();
        TWARNING << "BevLanePostImpl::AdjacentLaneProcess: Left left lane a0 " << left_left_a0 << " invalid, reset...";
      }
    }
  }

  // right right是否依然存在且未被占用
  if (right_right_tracked_) {
    TDEBUG << "trying update right right...";
    if (right_right_target_->GetLanePosIndex() == LaneLinePositionIndex::LEFT_1 ||
        right_right_target_->GetLanePosIndex() == LaneLinePositionIndex::RIGHT_1) {
      TDEBUG << "right right occupied, reset...";
      ResetRightRightLane();
    } else {
      auto right_right_a0 = right_right_target_->GetFitA0();
      // 判断right right是否依然合法
      if (CheckRRLinePositive(right_right_target_)) {
        TDEBUG << "right right updated " << right_right_a0;
      } else {
        ResetRightRightLane();
        TWARNING << "BevLanePostImpl::AdjacentLaneProcess: Right right lane a0 " << right_right_a0
                 << " invalid, reset...";
      }
    }
  }

  if (!left_left_tracked_) {
    TDEBUG << "trying init left left...";
    ResetLeftLeftLane();
    InitLeftLeftLane();
  }
  if (!right_right_tracked_) {
    TDEBUG << "trying init right right...";
    ResetRightRightLane();
    InitRightRightLane();
  }
}

bool BevLanePostImpl::CheckLLLinePositive(const LaneTrackletPtr& lane_target) {
  float a0 = lane_target->GetFitA0();
  float a0_limit_max = 7.0;
  float a0_limit_min = 3.5;
  if (ego_left_tracked_) {
    a0_limit_min = ego_left_target_->GetFitA0() + float(2.5);
    a0_limit_max = ego_left_target_->GetFitA0() + float(4.5);
  }
  if (a0 >= a0_limit_min && a0 <= a0_limit_max) {
    return true;
  }
  return false;
}

bool BevLanePostImpl::CheckRRLinePositive(const LaneTrackletPtr& lane_target) {
  float a0 = lane_target->GetFitA0();
  float a0_limit_max = -3.5;
  float a0_limit_min = -7.0;
  if (ego_right_tracked_) {
    a0_limit_max = ego_right_target_->GetFitA0() - float(2.5);
    a0_limit_min = ego_right_target_->GetFitA0() - float(4.5);
  }
  if (a0 >= a0_limit_min && a0 <= a0_limit_max) {
    return true;
  }
  return false;
}

void BevLanePostImpl::InitLeftLeftLane() {
  if (tracklets_.empty()) {
    TDEBUG << "Left left init failed. no mature lane!";
    return;
  }
  if (iter_right_1_ - tracklets_.begin() >= 1) {
    for (auto iter_left = iter_right_1_ - 1; iter_left >= tracklets_.begin(); iter_left--) {
      const auto& lane_target = *iter_left;
      if (lane_target->GetLanePosIndex() != LaneLinePositionIndex::UNINITED_INDEX) {
        continue;
      }
      if (CheckLLLinePositive(lane_target)) {
        TDEBUG << "Init Left Left lane success, a0:" << lane_target->GetFitA0();
        SetLeftLeftLane(lane_target);
        break;
      }
    }
  }
}

void BevLanePostImpl::InitRightRightLane() {
  if (tracklets_.empty()) {
    TDEBUG << "Right Right init failed. no mature lane!";
    return;
  }
  if (tracklets_.end() - iter_right_1_ >= 1) {
    for (auto iter_right = iter_right_1_; iter_right < tracklets_.end(); iter_right++) {
      const auto& lane_target = *iter_right;
      if (lane_target->GetLanePosIndex() != LaneLinePositionIndex::UNINITED_INDEX) {
        continue;
      }
      if (CheckRRLinePositive(lane_target)) {
        TDEBUG << "Init Right Right lane success, a0:" << lane_target->GetFitA0();
        SetRightRightLane(lane_target);
        break;
      }
    }
  }
}
// ************************** 车道线holdon处理 **************************
void BevLanePostImpl::EgoLaneHoldOnProcess() {
  // can only be activated when lane_position_process_ is on
  if (!lane_position_select_) {
    TWARNING << "egolane hold on failed, the lane_position_process is off";
    return;
  }

  for (const auto& tracklet : tracklets_) {
    if (ego_left_tracked_ && tracklet->GetTrackletId() == ego_left_target_->GetTrackletId()) {
      tracklet->SetHoldOn(true);
      tracklet->SetFitNum(1);
    } else if (ego_right_tracked_ && tracklet->GetTrackletId() == ego_right_target_->GetTrackletId()) {
      tracklet->SetHoldOn(true);
      tracklet->SetFitNum(1);

    } else {
      tracklet->SetHoldOn(false);
    }
  }
}

void BevLanePostImpl::GenerateTracklets(std::vector<LaneLineVision>& lanes_tracked) {
  lanes_tracked.clear();
  lanes_tracked.reserve(tracklets_.size());

  for (const auto& tracklet : tracklets_) {
    bool is_mature = tracklet->IsHoldOn() || (tracklet->IsTracked() && tracklet->IsGrownUp() &&
                                              tracklet->GetHits() >= min_hits_ && tracklet->IsUpdateValid());
    if (is_mature) {
      auto res = tracklet->GetLaneWithFusedPts();
      if (res.OriginPointsWorld.size() < min_world_pts_num_thresh_ ||
          res.OriginPointsWorld.front().x > max_start_dist_thresh_) {
        TDEBUG << "DELETE ONE " << res.OriginPointsWorld.size() << " " << res.OriginPointsWorld.front().x << " "
               << res.OriginPointsWorld.front().y;
        continue;
      }
      lanes_tracked.push_back(res);
    }
  }

  if (lane_parallelization_ && lanes_tracked.size() >= 2) {
    std::vector<int> valid_lane_idxs;
    // get line pts
    std::vector<std::vector<std::pair<float, float>>> points;
    for (size_t idx = 0; idx < lanes_tracked.size(); idx++) {
      // todo cjs
      const auto& obj = lanes_tracked[idx];
      std::vector<std::pair<float, float>> lane_pts;
      int near_x = 0;
      int far_x = 0;
      if (fabs(obj.a0) < 4) {
        near_x = egolane_parallel_fit_near_x_;
        far_x = egolane_parallel_fit_far_x_;
      } else {
        near_x = 25;
        far_x = 35;
      }
      for (size_t i = 0; i < obj.OriginPointsWorld.size(); ++i) {
        if (obj.OriginPointsWorld[i].x >= near_x && obj.OriginPointsWorld[i].x <= far_x) {
          lane_pts.push_back({obj.OriginPointsWorld[i].x, obj.OriginPointsWorld[i].y});
        }
      }
      if (lane_pts.size() > 3) {
        valid_lane_idxs.push_back(idx);
        points.emplace_back(lane_pts);
      } else {
        TDEBUG << "JUMP ONE LANE";
      }
    }

    // point spasify
    for (auto& point : points) {
      point = SparsifyPoints(point);
    }

    // get parallelized params
    std::vector<ld_algo::PolynomialCoefficients> parallel_fit_params = ld_algo::FitParallelLines(points, false, 3);

    for (size_t i = 0; i < valid_lane_idxs.size(); i++) {
      auto& line = lanes_tracked[valid_lane_idxs[i]];
      auto& param = parallel_fit_params[i];
      line.a0 = param.a0;
      line.a1 = param.a1;
      line.a2 = param.a2;
      line.a3 = param.a3;
    }
  }
  TDEBUG << "GenerateTracklets: " << lanes_tracked.size() << " lanes";
}

std::vector<std::pair<float, float>> BevLanePostImpl::SparsifyPoints(
    const std::vector<std::pair<float, float>>& points) {
  if (points.size() < 10) {
    return points;
  }
  std::vector<std::pair<float, float>> filtered_points;
  if (points.empty()) return filtered_points;

  // Initialize the first point
  filtered_points.push_back(points.front());
  for (int i = 0; i < 5; ++i) {
    filtered_points.push_back(points[i]);
  }

  // Iterate through the rest of the points
  float last_x = points.front().first;
  for (const auto& point : points) {
    if (point.first - last_x >= 1.0f) {
      filtered_points.push_back(point);
      last_x = point.first;
    }
  }
  return filtered_points;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
