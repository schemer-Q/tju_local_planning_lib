/**
 * @file yaw_fusion_kf_impl.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-11-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/algo/track/kalman_filter/linear_kalman_filter.h"
#include "trunk_perception/app/target_fusion/data_fusion/yaw_fusion_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

struct YawFusionKFConfig : public YawFusionConfig {
  typedef std::shared_ptr<YawFusionKFConfig> Ptr;
  typedef std::shared_ptr<const YawFusionKFConfig> ConstPtr;
};

/**
 * @brief 仅使用激光雷达数据的航向角融合实现,基于KF
 * @note 效果不好！！！
 *
 */
class YawFusionKFImpl : public YawFusionBase {
 public:
  YawFusionKFImpl(const YawFusionConfig::ConstPtr& config);

  std::uint32_t Init(const float& yaw) override;

  void Predict(const double& dt) override;

  std::uint32_t Update(const float& yaw) override;

 private:
  std::shared_ptr<LinearKalmanFilter> motion_filter_ = nullptr;
  float default_dt_ = 0.1;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
