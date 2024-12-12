/**
 * @file od_sparse4d_impl.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 基于Sparse4D的物体检测实现
 * @version 0.1
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/app/object_detection/od_base.h"

namespace net {
class ObjectDetector;
struct Sparse4dWithDistortionInput;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 基于Sparse4D的物体检测实现
 */
class OdSparse4DImpl : public OdBase {
 public:
  OdSparse4DImpl();
  ~OdSparse4DImpl() override;

  std::uint32_t Init(const YAML::Node& config) override;
  std::uint32_t Run(const double& ts = 0.0) override;
  std::any GetData(const std::string& key) override;

 private:
  std::shared_ptr<net::ObjectDetector> detector_ = nullptr;
  net::Sparse4dWithDistortionInput* input_ = nullptr;
  std::string model_config_path_;
  std::vector<std::string> cameras_;
  bool camera_params_initialized_ = false;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END