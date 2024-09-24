/**
 * @file preprocess_base.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief lidar preprocess base class
 * @version 0.1
 * @date 2024-09-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/common/macros.h"
#include "trunk_perception/app/base/app_base.h"
#include "trunk_perception/common/types/point.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 激光雷达预处理
 * @details 基类中提供点云tf变换
 * @details 如果需要添加和修改功能，可以通过继承该类，重写相关函数
 */
class PreprocessBase : public AppBase {
 public:

  PreprocessBase();
  ~PreprocessBase();

  std::uint32_t Init(const YAML::Node& config) override;
  std::uint32_t Run(const double& ts) override;
  std::any GetData(const std::string& key) override;

 protected:
  PointCloudPtr TransformPointCloud(const PointCloudConstPtr& cloud, const Eigen::Isometry3f& tf);

 private:
  std::string lidar_name_ = "";
  PointCloudPtr tf_cloud_ = nullptr;
  
};


TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
