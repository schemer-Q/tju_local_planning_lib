#include "trunk_perception/app/target_fusion/data_fusion/yaw_fusion_kf_impl.h"
#include "trunk_perception/common/error/code.hpp"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

YawFusionKFImpl::YawFusionKFImpl(const YawFusionConfig::ConstPtr& config) : YawFusionBase(config) {
  // clang-format off
  Eigen::MatrixXd A = (Eigen::MatrixXd(2, 2) <<
                      1, default_dt_,
                      0, 1).finished();
  Eigen::MatrixXd H = (Eigen::MatrixXd(1, 2) << 1, 0).finished();
  Eigen::MatrixXd P = (Eigen::MatrixXd(2, 2) <<
                      100, 0,
                      0, 100).finished();
  Eigen::MatrixXd Q = (Eigen::MatrixXd(2, 2) <<
                      100, 0,
                      0, 100).finished();
  Eigen::MatrixXd R = (Eigen::MatrixXd(1, 1) << 0.1).finished();

  // clang-format on
  motion_filter_ = std::make_shared<LinearKalmanFilter>(A, H, P, Q, R);
}

std::uint32_t YawFusionKFImpl::Init(const float& yaw) {
  Eigen::VectorXd init_state(2);
  init_state << yaw, 0;
  motion_filter_->init(init_state);
  return ErrorCode::SUCCESS;
}

void YawFusionKFImpl::Predict(const double& dt) {
  Eigen::MatrixXd A = motion_filter_->getStateTransitionMatrix();
  A(0, 1) = dt;
  motion_filter_->setStateTransitionMatrix(A);
  motion_filter_->predict();
  yaw_ = motion_filter_->getState()(0);
}

std::uint32_t YawFusionKFImpl::Update(const float& yaw) {
  Eigen::VectorXd z(1);
  z << yaw;
  motion_filter_->update(z);
  yaw_ = motion_filter_->getState()(0);
  return ErrorCode::SUCCESS;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
