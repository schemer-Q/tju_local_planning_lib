/**
 * @file fod_vision_sdk.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-11-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/app/object_detection_front_vision/fod_vision_base.h"
#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/tools/camera_undistort.hpp"
#include "trunk_perception/common/tools/standard_camera_projection.hpp"

#include "SDKVisDetR3DDconfFront/SDKVisDetR3DDconfFront.h"
#include "SDKVisDetR3DDconfFront/VisDetR3DDconfFrontData.h"

#include <unordered_map>

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

class FodVisionSDK : public FodVisionBase {
 public:
  FodVisionSDK() = default;
  ~FodVisionSDK() override;

  std::uint32_t Init(const YAML::Node& config) override;
  std::uint32_t Run(const double& ts = 0.0) override;
  std::any GetData(const std::string& key) override;

 private:
  uint32_t InitCameraParams();
  void postProcess(const std::shared_ptr<FodVisionFrame>& fod_vision_frame);

 private:
  std::string name_;         ///< 传感器名称
  std::string type_;         ///< 图像类型，e.g. "undistorted"
  std::string config_file_;  ///< 模型配置文件路径

  std::shared_ptr<CameraUndistort> camera_undistort_ = nullptr;
  std::shared_ptr<StandardCameraProjection> camera_projection_ = nullptr;

  r3d::VisDetR3DDconfFrontData* r3d_data_ = nullptr;          ///< 前向视觉检测数据, 裸指针，注意释放
  r3d::SDKVisdeta5R3DInst* r3d_instance_ = nullptr;           ///< 前向视觉检测实例, 裸指针，注意释放
  std::shared_ptr<r3d::CameraParam> camera_param_ = nullptr;  ///< 相机参数, SDK中的结构
  bool lazy_init_ = false;  ///< 是否懒初始化SDK,SDK初始化需要相机参数，后续可以考虑优化SDK
  bool is_init_ = false;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END