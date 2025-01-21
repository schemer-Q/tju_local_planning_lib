/**
 * @file od_mm_sparse4d_impl.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 多模态Sparse4D物体检测实现
 * @version 0.1
 * @date 2025-01-21
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "trunk_perception/app/object_detection/od_base.h"

namespace net {
class ObjectDetector;
struct PointPillarSparse4dInput;
}  // namespace net

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 多模态Sparse4D物体检测实现
 */
class OdMMSparse4DImpl : public OdBase {
 public:
  OdMMSparse4DImpl();
  ~OdMMSparse4DImpl() override;

  std::uint32_t Init(const YAML::Node& config) override;
  std::uint32_t Run(const double& ts = 0.0) override;
  std::any GetData(const std::string& key) override;

 private:
  std::shared_ptr<net::ObjectDetector> detector_ = nullptr;
  net::PointPillarSparse4dInput* input_ = nullptr;
  std::string model_config_path_;
  bool camera_params_initialized_ = false;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END