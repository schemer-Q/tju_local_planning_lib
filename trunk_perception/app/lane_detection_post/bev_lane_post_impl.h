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
#include "trunk_perception/app/lane_detection_post/algo/hungarian.h"
#include "trunk_perception/app/lane_detection_post/algo/id_pool.h"
#include "trunk_perception/app/lane_detection_post/algo/lane_tracklet.h"
#include "trunk_perception/app/lane_detection_post/ld_post_base.h"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/lane.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

class BevLanePostImpl : public LdPostBase {
 public:
  using LaneTrackletPtr = std::shared_ptr<ld_post::LaneTracklet>;

  BevLanePostImpl();
  ~BevLanePostImpl() override;

  std::uint32_t Init(const YAML::Node& config) override;
  std::uint32_t Run(const double& ts) override;
  std::any GetData(const std::string& key) override;

 private:
  /**
   * @brief 估计车道线拟合多项式的次数
   * @details 存在遮挡的情况下，高次方程拟合结果不稳定。在车速低或曲率统计比较小的情况下会使用一次方程拟合
   * @details 依赖VehicleInfo中的车速信息，如果信息获取失败，则使用高次方程拟合
   */
  void EstimatePolyFittingNum();

  /**
   * @brief 对已有的tacklet进行预测，同时将当前车辆位姿传进tacklet中
   *
   */
  void Predict();

  void Associate(const std::vector<LaneLineVision>& lanes_detected);

  void Update(const std::vector<LaneLineVision>& lanes_detected);

  /**
   * @brief 清除无效tracklet
   *
   */
  void ClearTracklet();

  void GenerateTracklets(std::vector<LaneLineVision>& lanes_tracked);

  float LaneSimilarity(const LaneTrackletPtr& tracklet, const LaneLineVision& lanes_detected);

  /**
   * @brief 设置ego left lane
   * @param ego_left ego left lane target
   */
  void SetEgoLeftLane(const LaneTrackletPtr& lane_target) {
    lane_target->SetLanePosIndex(LaneLinePositionIndex::LEFT_1);
    ego_left_tracked_ = true;
    ego_left_target_ = lane_target;
  }

  /**
   * @brief 重置ego left lane
   */
  void ResetEgoLeftLane() {
    ego_left_tracked_ = false;
    ego_left_target_ = nullptr;
    for (auto& tracklet : tracklets_) {
      if (tracklet->GetLanePosIndex() == LaneLinePositionIndex::LEFT_1) {
        tracklet->SetLanePosIndex(LaneLinePositionIndex::UNINITED_INDEX);
      }
    }
  }

  /**
   * @brief 设置ego right lane
   * @param ego_right ego right lane target
   */
  void SetEgoRightLane(const LaneTrackletPtr& lane_target) {
    lane_target->SetLanePosIndex(LaneLinePositionIndex::RIGHT_1);
    ego_right_tracked_ = true;
    ego_right_target_ = lane_target;
  }

  /**
   * @brief 重置ego right lane
   */
  void ResetEgoRightLane() {
    ego_right_tracked_ = false;
    ego_right_target_ = nullptr;
    for (auto& tracklet : tracklets_) {
      if (tracklet->GetLanePosIndex() == LaneLinePositionIndex::RIGHT_1) {
        tracklet->SetLanePosIndex(LaneLinePositionIndex::UNINITED_INDEX);
      }
    }
  }

  /**
   * @brief 设置left_left lane
   * @param left_left left_left lane target
   */
  void SetLeftLeftLane(const LaneTrackletPtr& lane_target) {
    lane_target->SetLanePosIndex(LaneLinePositionIndex::LEFT_2);
    left_left_tracked_ = true;
    left_left_target_ = lane_target;
  }

  /**
   * @brief 重置left left lane
   */
  void ResetLeftLeftLane() {
    left_left_tracked_ = false;
    left_left_target_ = nullptr;
    for (auto& tracklet : tracklets_) {
      if (tracklet->GetLanePosIndex() == LaneLinePositionIndex::LEFT_2) {
        tracklet->SetLanePosIndex(LaneLinePositionIndex::UNINITED_INDEX);
      }
    }
  }

  /**
   * @brief 设置right_right lane
   * @param right_right lane target
   */
  void SetRightRightLane(const LaneTrackletPtr& lane_target) {
    lane_target->SetLanePosIndex(LaneLinePositionIndex::RIGHT_2);
    right_right_tracked_ = true;
    right_right_target_ = lane_target;
  }

  /**
   * @brief 重置right_right lane
   */
  void ResetRightRightLane() {
    right_right_tracked_ = false;
    right_right_target_ = nullptr;
    for (auto& tracklet : tracklets_) {
      if (tracklet->GetLanePosIndex() == LaneLinePositionIndex::RIGHT_2) {
        tracklet->SetLanePosIndex(LaneLinePositionIndex::UNINITED_INDEX);
      }
    }
  }

  /**
   * @brief 车道线位置处理逻辑
   *
   */
  void LanePositionProcess();

  /**
   * @brief ego lane后处理逻辑
   */
  void EgoLaneProcess() {
    // ego lane 更新
    EgoLaneUpdate();
    // ego lane 校验
    EgoLaneCheck();
    // ego lane refit
    EgoLaneRefit();
    // ego lane filter
    // EgoLaneFilter();
    // ego lane protect
    EgoLaneProtect();
  }

  /**
   * @brief ego lane更新逻辑
   */
  void EgoLaneUpdate();

  /**
   * @brief ego lane 校验
   */
  void EgoLaneCheck();

  /**
   * @brief ego lane 平行拟合
   * @return
   */
  void EgoLaneRefit();

  // /**
  //  * @brief ego lane 滤波
  //  * @return
  //  */
  // void EgoLaneFilter();

  /**
   * @brief ego lane 保护
   */
  void EgoLaneProtect();

  /**
   * @brief 初始化ego lane
   * @return
   */
  bool InitEgoLane();

  /**
   * @brief 仅左侧ego lane存在的宽松校验
   * @return 0:bad; 1:middle; 2:good; 3:best.
   */
  int CheckEgoLaneLeftQualityLoosely();

  /**
   * @brief 仅右侧ego lane存在的宽松校验
   * @return 0:bad; 1:middle; 2:good; 3:best.
   */
  int CheckEgoLaneRightQualityLoosely();

  /**
   * @brief 两侧ego lane同时存在的宽松校验
   * @return 0:bad; 1:middle; 2:good; 3:best.
   */
  int CheckEgoLaneUnionQualityLoosely();

  /**
   * @brief 存在一侧ego lane，尝试match另一侧
   * @param lane_index 需要match的车道线位置
   * @return
   */
  bool TryingMatchEgoPairLane(LaneLinePositionIndex lane_index);

  /**
   * @brief 验证一组车道线是否满足组成ego lane的条件
   * @param lane_left  左侧车道线
   * @param lane_right 右侧车道线
   * @return
   */
  static bool EgoPairLaneMatch(const LaneTrackletPtr& lane_left, const LaneTrackletPtr& lane_right);

  /**
   * @brief 变道判断
   * @return true:发生变道 false:车道保持中
   */
  bool IsChangeLane();

  /**
   * @brief 左左和右右车道线处理逻辑
   */
  void AdjacentLaneProcess();

  /**
   * @brief left left 车道线校验
   * @param lane_target
   * @return
   */
  bool CheckLLLinePositive(const LaneTrackletPtr& lane_target);

  /**
   * @brief right right 车道线校验
   * @param lane_target
   * @return
   */
  bool CheckRRLinePositive(const LaneTrackletPtr& lane_target);

  /**
   * @brief 初始化left left lane
   */
  void InitLeftLeftLane();

  /**
   * @brief 初始化right right lane
   */
  void InitRightRightLane();

  void EgoLaneHoldOnProcess();

  std::vector<std::pair<float, float>> SparsifyPoints(const std::vector<std::pair<float, float>>& points);

  std::vector<LaneTrackletPtr> tracklets_;     ///< 车道线跟踪序列
  std::vector<LaneLineVision> dropped_lanes_;  ///< 记录被clear掉的无效车道线

  Eigen::Matrix4d cur_pose_;  ///< 当前帧自车位姿

  std::vector<int> associate_tracklet_ids_;  ///< 关联的跟踪序列在tracklets_中的索引
  std::vector<int> associate_det_ids_;       ///< 关联的检测序列在LDFrame.lanes_detected中的索引
  ld_post::HungarianAlgorithm hungarian_algo_;

  std::unique_ptr<ld_post::IDPool> id_pool_;

  // specified target
  // node:使用Set和Reset方法以确保target指针和tracked标记被同步设置，如SetRightRightLane()和ResetRightRightLane()
  LaneTrackletPtr ego_left_target_ = nullptr;
  LaneTrackletPtr ego_right_target_ = nullptr;
  LaneTrackletPtr left_left_target_ = nullptr;
  LaneTrackletPtr right_right_target_ = nullptr;
  bool ego_left_tracked_ = false;
  bool ego_right_tracked_ = false;
  bool left_left_tracked_ = false;
  bool right_right_tracked_ = false;

  // 右侧第一条车道线的迭代器
  std::vector<LaneTrackletPtr>::iterator iter_right_1_;

  // super parameters
  std::string camera_name_;
  int num_anchor_ = 9;                ///< 计算iou时，anchor的数量
  float iou_thresh_ = 0.4;            ///< 计算iou时，iou阈值
  size_t max_id_ = 254;               ///< 跟踪序列ID池的最大值
  bool lane_position_select_ = true;  ///< 是否执行车道线的位置后处理逻辑
  bool egolane_hold_on_mode_ = true;  ///< 遮挡情况下的holdon模式开关
  size_t min_hits_ = 5;               ///< 最小观测次数
  uint min_world_pts_num_thresh_ = 10;  ///< 车道线最小点数阈值，小于则无效
  float max_start_dist_thresh_ = 15;    ///< 车道线起始点距离阈值，大于则无效
  bool lane_parallelization_ = true;    ///< 是否进行车道线平行化
  float egolane_parallel_fit_near_x_ = 0;
  float egolane_parallel_fit_far_x_ = 100;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
