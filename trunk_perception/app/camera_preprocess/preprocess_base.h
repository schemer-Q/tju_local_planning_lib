/**
 * @file preprocess_base.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 图像预处理应用
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/app/base/app_base.h"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/image.h"


TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

/**
 * @brief 图像预处理
 * @details 基类中提供图像去畸变
 * @details 如果需要添加和修改功能，可以通过继承该类，重写相关函数
 */
class CameraPreprocessBase : public AppBase {
 public:
  CameraPreprocessBase();
  ~CameraPreprocessBase();

  std::uint32_t Init(const YAML::Node& config) override;
  std::uint32_t Run(const double& ts) override;
  std::any GetData(const std::string& key) override;

 private:
  std::string camera_name_ = "";
  std::shared_ptr<Image> undistort_image_ = nullptr;
};


TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
