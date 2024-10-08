/**
 * @file ld_SDKBevLaneConeDet.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 基于 SDKBevLaneConeDetHighway 的车道线检测器
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <cstdint>
#include <memory>
#include "SDKBevLaneConeDetHighway/BevLaneConeDetData.h"
#include "SDKBevLaneConeDetHighway/SDKBevLaneConeDet.h"

#include "trunk_perception/app/lane_detection/algo/Ransac.h"
#include "trunk_perception/app/lane_detection/algo/lane_clustering.h"
#include "trunk_perception/app/lane_detection/lane_detector_base.h"
#include "trunk_perception/common/data_manager/data_wrapper/ld_frame.h"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/tools/camera_undistort.hpp"
#include "trunk_perception/common/tools/standard_camera_projection.hpp"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

class LaneDetectorSDKBevLaneConeDet : public LaneDetectorBase {
 public:
  LaneDetectorSDKBevLaneConeDet() = default;
  ~LaneDetectorSDKBevLaneConeDet();

  std::uint32_t Init(const YAML::Node& config) override;
  std::uint32_t Run(const double& ts = 0.0) override;
  std::any GetData(const std::string& key) override;

 private:
  uint32_t InitCameraParams();

  void LanePost(const std::shared_ptr<LDFrame>& ld_frame);

  std::string name_;         ///< 传感器名称
  std::string type_;         ///< 图像类型，e.g. "undistorted"
  std::string config_file_;  ///< 模型配置文件路径

  // ransac 参数
  int ransac_max_iterations_ = 5;
  float ransac_inlier_pts_rate_ = 0.8;
  int ransac_degree_ = 3;

  // 其他后处理参数
  size_t min_cone_margin_pts_threshold_ = 5;
  float bev_lane_pt_min_interval_ = 2.0;     ///< bev车道线两点之间的最小间隔
  size_t detected_lane_min_pts_ = 10;        ///< 车道线检测点数最小限制
  float detected_lane_min_length_x_ = 20.0;  ///< 车道线远端点的最小距离限制
  float detected_lane_min_range_x_ = 20.0;   ///< 车道线检测长度最短限制
  size_t lane_bev_fitting_exponent_ = 3;     ///< bev车道线拟合多项式指数

  std::shared_ptr<CameraUndistort> camera_undistort_ = nullptr;
  std::shared_ptr<StandardCameraProjection> camera_projection_ = nullptr;

  std::shared_ptr<ld_algo::RansacPolynomialFitter> ransac_fitter_ = nullptr;
  ld_algo::LaneClustering lane_cluster_;

  bevlaneconedet::SDKBevLaneConeDetInst* bevlane_instance_ = nullptr;  ///< 车道线检测实例, 裸指针，注意释放
  bevlaneconedet::BevLaneConeDetData* bevlane_data_ = nullptr;  ///< 车道线检测数据, 裸指针，注意释放
  std::shared_ptr<bevlaneconedet::CameraParam> camera_param_ = nullptr;  ///< 相机参数, SDK中的结构
  bool lazy_init_ = false;  ///< 是否懒初始化SDK,SDK初始化需要相机参数，后续可以考虑优化SDK
  bool is_init_ = false;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END