/**
 * @file target_fusion_a.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 目标后融合A类
 * @version 0.1
 * @date 2024-10-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <cstdint>
#include <list>
#include <memory>
#include <vector>
#include "trunk_perception/app/target_fusion/data_associate/tracker_objects_match.h"
#include "trunk_perception/app/target_fusion/data_fusion/tracker.h"
#include "trunk_perception/app/target_fusion/data_fusion/tracker_manager.h"
#include "trunk_perception/app/target_fusion/debug_data.h"
#include "trunk_perception/app/target_fusion/target_fusion_base.h"
#include "trunk_perception/app/target_fusion/utils/id_pool.h"
#include "trunk_perception/common/data_manager/data_wrapper/target_fusion_frame.h"
#include "trunk_perception/common/types/odometry.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

class TargetFusionA : public TargetFusionBase {
 public:
  TargetFusionA();
  ~TargetFusionA() override;

  std::uint32_t Init(const YAML::Node& config) override;
  std::uint32_t Run(const double& ts) override;
  std::any GetData(const std::string& key) override;

 private:
  /**
   * @brief 获取输入数据
   *
   * @param ts 时间戳
   * @return std::uint32_t 错误码
   */
  std::uint32_t GetInputData(const double& ts);

  /**
   * @brief 获取激光雷达数据
   *
   * @return uint32_t 错误码
   */
  uint32_t GetLidarData();

  /**
   * @brief 获取前向毫米波雷达数据, 转到车体系下，并补偿到激光雷达时间
   *
   * @return uint32_t 错误码
   */
  uint32_t GetFrontRadarData();

  /**
   * @brief 获取前向视觉数据
   * 
   * @return uint32_t 错误码
   * 
   * @author zzg 2024-12-13
   */
  uint32_t GetFrontVisionData();

  /**
   * @brief 获取odometry数据
   *
   * @param ts [IN] 需要对齐的时间戳, 秒
   * @param odometry_ptr [OUT] 获取到的odometry数据指针
   * @param use_compensation [IN] 是否进行时间补偿
   * @return uint32_t 错误码
   */
  uint32_t GetOdometryData(const double& ts, std::shared_ptr<Odometry>& odometry_ptr,
                           const bool& use_compensation = false);

  /**
   * @brief 转到局部坐标系下(dr)
   *
   */
  void ConvertToLocalCoordinate();

  /**
   * @brief 将跟踪序列预测到当前帧时间
   *
   */
  void Predict();

  /**
   * @brief 将tracks与检测结果进行关联
   *
   */
  void Association();

  /**
   * @brief 对未关联上的各个观测源的目标创建新的跟踪器
   *
   */
  void GenerateNewTrackers();

  /**
   * @brief 将idx映射回原idx
   *
   * @param idx_map 映射关系，idx_map[i] = j, 代表現在index=i，原index为j
   * @param association_result 关联结果
   */
  void ConvertIdx(const std::vector<size_t>& idx_map, AssociationResult& association_result);

  void Update();

  /**
   * @brief 航迹管理
   *
   */
  void GateKeeper();

  void GenerateAssociateDebugData();

  /**
   * @brief 给下游的输出
   * 
   */
  void GenerateFusedObject();

  std::shared_ptr<common::TargetFusionFrame> frame_ptr_ = nullptr;

  bool if_time_compensate_lidar_odometry_ = true;  ///< 是否对激光雷达odometry进行补偿
  bool if_time_compensate_front_radar_ = true;     ///< 是否对毫米波雷达数据进行时间补偿
  bool if_space_compensate_front_radar_ = true;  ///< 是否对毫米波雷达数据进行空间补偿(补偿自车位移)
  bool if_time_compensate_front_vision_odometry_ = true;  ///< 是否对前向视觉数据进行时间补偿

  bool is_first_frame_ = true;  ///< 是否是第一帧，第一帧则直接创建新的tracker，不需要关联
  std::shared_ptr<IDPool> id_pool_ptr_ = nullptr;
  std::shared_ptr<TrackerManager> tracker_manager_ptr_ = nullptr;
  std::shared_ptr<TrackerObjectsMatch> tracker_objects_match_ptr_ = nullptr;

  AssociationResult stable_tracker_lidar_association_result_;
  AssociationResult stable_tracker_radar_association_result_;
  AssociationResult stable_tracker_front_vision_association_result_;

  AssociationResult new_tracker_lidar_association_result_;
  AssociationResult new_tracker_radar_association_result_;
  AssociationResult new_tracker_front_vision_association_result_;

  AssociationResult lost_tracker_lidar_association_result_;
  AssociationResult lost_tracker_radar_association_result_;
  AssociationResult lost_tracker_front_vision_association_result_;

  std::vector<TrackerPtr> new_trackers_;  ///< 起始集, 跟踪帧数低于阈值，连续多帧丢失观测后直接删除，不进入丢失集
  std::vector<TrackerPtr> stable_trackers_;  ///< 稳定集，同时也是输出集合
  std::vector<TrackerPtr> lost_trackers_;  ///< 丢失集，来自稳定集丢失多帧观测后的跟踪序列，仍会保留以便与新增观测匹配上

  // debug data
  AssociateDebugData::Ptr associate_debug_data_ptr_;

  // 参数
  int new_to_stable_life_thresh_ = 3;   ///< 起始集进入稳定集跟踪帧数阈值,否则直接删除
  int stable_to_lost_life_thresh_ = 5;  ///< 稳定集进入丢失集跟踪帧数阈值
  int lost_to_stable_life_thresh_ = 3;  ///< 丢失集进入稳定集跟踪帧数阈值
  int lost_to_delete_life_thresh_ = 10;  ///< 丢失集删除跟踪帧数阈值
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END