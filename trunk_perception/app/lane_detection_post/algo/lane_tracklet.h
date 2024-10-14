/**
 * @file lane_tracker.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 车道线跟踪片段
 * @version 0.1
 * @date 2024-10-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <cstddef>
#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include "trunk_perception/app/lane_detection_post/algo/type_filter.h"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/tools/camera_undistort.hpp"
#include "trunk_perception/common/tools/standard_camera_projection.hpp"
#include "trunk_perception/common/types/lane.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_post {

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

const int N_ANCHOR_POINTS = 9;

/**
 * @brief 用于多帧融合的检测点
 *
 */
struct FusionPt {
  int age{};
  cv::Point3f loc;
  cv::Point3f init_loc;

  FusionPt() = delete;

  FusionPt(int age, const cv::Point3f& loc) : age(age), loc(loc), init_loc(loc) {}
};

class LaneTracklet {
 public:
  LaneTracklet() = delete;
  /**
   * @brief 构造函数
   *
   * @param tracklet_id 新跟踪序列的ID
   * @param lane 车道线
   * @param cur_pose 自车位姿
   * @param camera_name 相机名称
   */
  LaneTracklet(const int& tracklet_id, const LaneLineVision& lane, const Eigen::Matrix4d& cur_pose,
               const std::string& camera_name);
  ~LaneTracklet();

  /**
   * @brief 给定自车位置，预测当前帧车道线位置
   * @param cur_pose (Eigen::Matrix4d) 当前帧自车位姿
   */
  void Predict(const Eigen::Matrix4d& cur_pose);

  /**
   * @brief 更新车道线
   *
   * @param det_lane 检测到的车道线
   */
  void Update(const LaneLineVision& det_lane);

  /**
   * @brief 设置多项式次数
   *
   * @param poly_num
   */
  void SetPolyNum(int poly_num) { poly_num_ = poly_num; }

  void ResetAssociateScore() { latest_lane_.associate_bev_score = 0.; }

  void SetAssociateScore(const float& score) { latest_lane_.associate_bev_score = score; }

  float GetAssociateScore() const { return latest_lane_.associate_bev_score; }

  /**
   * @brief 获取使用融合点更新anchor后的车道线
   *
   * @return LaneLineVision
   */
  LaneLineVision GetLaneWithFusedPts();

  /**
   * @brief 获取多项式系数
   *
   * @return std::vector<float>
   */
  std::vector<float> GetLaneFitParam() const {
    return {
        latest_lane_.a0,
        latest_lane_.a1,
        latest_lane_.a2,
        latest_lane_.a3,
    };
  }

  float GetFitA0() const { return latest_lane_.a0; }

  int GetTrackletId() const { return tracklet_id_; }

  bool IsLost();

  LaneLinePositionIndex GetLanePosIndex() const { return latest_lane_.lane_line_position; }

  void SetLanePosIndex(LaneLinePositionIndex lane_pos_index) { latest_lane_.lane_line_position = lane_pos_index; }

  bool IsTracked() const { return lost_age_ <= 0; }

  bool IsGrownUp() const { return age_ > 1; }

  size_t GetHits() const { return hits_; }

  bool IsUpdateValid() const { return update_valid_; }

  std::vector<cv::Point3f> GetRefitPts() const { return refit_pts_; }

  void SetFitParam(float a0, float a1, float a2, float a3) {
    latest_lane_.a0 = a0;
    latest_lane_.a1 = a1;
    latest_lane_.a2 = a2;
    latest_lane_.a3 = a3;
  }

  bool IsHoldOn() const { return is_hold_on_; }
  void SetHoldOn(bool is_hold_on) { is_hold_on_ = is_hold_on; }

  void SetFitNum(int num) { poly_num_ = num; }

 private:
  /**
   * @brief fusion pts 融合
   *
   */
  void InitFusionPtGridOccupancy();

  /**
   * @brief 使用当前lane的车道线点初始化FusionPts，要求OriginPointsWorld按照纵向距离从近到远排列
   *
   */
  void InitFusionPts();

  /**
   * @brief 初始化anchor.x
   *
   */
  void InitAnchors();

  /**
   * @brief 初始化kalman滤波器，包括anchor.y和系数的kalman滤波器
   *
   */
  void InitKalmanFilter();

  /**
   * @brief 初始化相机投影矩阵
   *
   */
  void InitCameraProjection();

  /**
   * @brief 将历史帧的pts推导至当前帧，并通过拟合更新后的点更新车道线方程
   * @param latset_to_cur_pose 上一帧到当前帧的位姿变换
   */
  void PredictFusionPts(const Eigen::Matrix4d& latset_to_cur_pose);

  /**
   * @brief 检测点是否满足阈值条件
   *
   * @param last_loc_vec
   * @param curr_loc_vec
   * @return true
   * @return false
   */
  bool CheckTransformPt(const Eigen::Vector4d& last_loc_vec, const Eigen::Vector4d& curr_loc_vec);

  std::vector<cv::Point3f> GenerateRefitPts();

  /**
   * @brief 根据refit_pts_拟合多项式系数
   *
   * @return true
   * @return false
   */
  bool RefitLaneCoeffs();

  /**
   * @brief 更新融合点
   *
   */
  void UpdateFusionPts();

  /**
   * @brief 在update阶段，检查融合点是否有效
   *
   * @param pt
   */
  bool CheckFusionPtValid(const FusionPt& pt);

  /**
   * @brief 使用kalman滤波器更新refit_pts_和latest_lane_
   *
   * @return int
   */
  int UpdateBevState();

  std::string camera_name_;  ///< 用户获取相机投影矩阵，将更新后的anchor投影到目标图像

  int tracklet_id_;
  LaneLineVision latest_lane_;   ///< 最新的车道线，检测 -> 预测 -> 更新
  Eigen::Matrix4d latest_pose_;  ///< 最新的自车位姿
  size_t age_ = 0;               ///< predict times，即更新的次数
  size_t hits_ = 0;              ///< update times,即存在关联观测的次数
  size_t lost_age_ = 0;          ///< 丢失关联观测的次数

  cv::KalmanFilter bev_anchor_kf_;  ///< 车道线在bev图像上的anchor点y跟踪滤波器, 9个y值
  cv::Mat bev_anchor_measurement_;  ///< 测量值
  cv::Mat bev_anchor_prediction_;   ///< 预测值

  cv::KalmanFilter coeff_kf_;  ///< 系数kalman滤波器
  cv::Mat coeff_measurement_;  ///< 测量值
  cv::Mat coeff_pred_;         ///< 预测值

  std::vector<int> occupied_x_grids_dense_;
  std::vector<int> occupied_x_grids_sparse_;

  std::vector<FusionPt>
      fusion_pts_;  ///< 包含历史帧和当前帧的点集， 历史点推导至当前帧后，根据age和位置是否合法来进行删除
  std::vector<cv::Point3f> refit_pts_;  ///< 从fusion_pts_中筛选出的合法点集，用于拟合多项式
  bool predict_with_history_ = false;   ///< 标识是否使用了历史帧来进行融合

  float near_x_;                  ///< 最近点x
  float far_x_;                   ///< 最远点x
  std::vector<float> x_anchors_;  ///< 每个anchor的x坐标

  std::deque<float> a0_queue_;   ///< a0的队列,用于统计0m处的方差
  std::deque<float> a70_queue_;  ///< a70的队列,用于统计70m处的方差

  int poly_num_ = 1;  ///< 多项式次数,允许被设置为1或3

  std::shared_ptr<StandardCameraProjection> camera_projection_ = nullptr;
  std::shared_ptr<CameraUndistort> camera_undistort_ = nullptr;

  bool update_valid_ = true;

  TypeFilter type_filter_{8};

  bool is_hold_on_ = false;  ///< allow to continually output the target in some special cases

  // super parameters
  int occupied_grid_x_start_ = 5;
  int occupied_grid_x_middle_ = 15;
  int occupied_grid_x_end_ = 80;
  float occupied_x_step_dense_ = 0.05;
  float occupied_x_step_sparse_ = 0.3;
  size_t queue_size_ = 5;  ///< 统计方差的队列长度
  float far_x_limit_ = 100.0;
  float near_x_limit_ = -10.0;
  float y_limit_ = 20.0;
  size_t fit_pt_num_limit_ = 5;
  int max_fusion_pt_age_ = 120000;  ///< 最大融合点年龄, 100min
};

};  // namespace ld_post

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
