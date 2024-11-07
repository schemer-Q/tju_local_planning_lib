#include "trunk_perception/app/target_fusion/data_fusion/kalman_motion_fusion.h"
#include <Eigen/src/Core/Matrix.h>
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

KalmanMotionFusion::KalmanMotionFusion(const MotionFusionConfig& config)
    : motion_model_(config.motion_model), m_sensor_R_(config.sensor_R), m_sensor_H_(config.sensor_H) {
  switch (motion_model_) {
    case MotionModel::CV:
      // clang-format off
      A_ = (Eigen::MatrixXd(4, 4) <<
            1, 0, default_dt_, 0,
            0, 1, 0, default_dt_,
            0, 0, 1, 0,
            0, 0, 0, 1).finished();
      P_ = (Eigen::MatrixXd(4, 4) <<
            0.1, 0,   5.0,    0,
            0,   0.1, 0,      5.0,
            5.0, 0,   1650.0, 0,
            0,   5.0, 0,      1650.0).finished();
      Q_ = (Eigen::MatrixXd(4, 4) <<
            0.15, 0,    0,    0,
            0,    0.15, 0,    0,
            0,    0,    0.35, 0,
            0,    0,    0,    0.35).finished();
      // clang-format on
      break;
  }
}

KalmanMotionFusion::~KalmanMotionFusion() = default;

bool KalmanMotionFusion::Init(const Eigen::VectorXd& x0) {
  if (x0.size() != A_.rows()) {
    TERROR << "[KalmanMotionFusion] init state error!";
    return false;
  }
  X_ = x0;
  initialized_ = true;
  return true;
}

bool KalmanMotionFusion::Predict(const double& dt) {
  if (!initialized_) {
    TFATAL << "[KalmanMotionFusion] not initialized!";
    return false;
  }

  A_(0, 2) = A_(1, 3) = dt;

  X_ = A_ * X_;
  P_ = A_ * P_ * A_.transpose() + Q_;
  return true;
}

bool KalmanMotionFusion::Update(const std::string& sensor_name, const Eigen::VectorXd& z) {
  if (!initialized_) {
    TFATAL << "[KalmanMotionFusion] not initialized!";
    return false;
  }

  if (m_sensor_R_.find(sensor_name) == m_sensor_R_.end() || m_sensor_H_.find(sensor_name) == m_sensor_H_.end()) {
    TFATAL << "[KalmanMotionFusion] sensor " << sensor_name << " not found!";
    return false;
  }

  const Eigen::MatrixXd& R = m_sensor_R_.at(sensor_name);
  const Eigen::MatrixXd& H = m_sensor_H_.at(sensor_name);

  const Eigen::MatrixXd K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
  X_ += K * (z - H * X_);
  P_ = (Eigen::MatrixXd::Identity(A_.rows(), A_.rows()) - K * H) * P_;
  return true;
}

Eigen::VectorXd KalmanMotionFusion::GetState() const { return X_; }

bool KalmanMotionFusion::SetSensorR(const std::string& sensor_name, const Eigen::MatrixXd& R) {
  if (m_sensor_R_.find(sensor_name) == m_sensor_R_.end()) {
    TERROR << "[KalmanMotionFusion] sensor " << sensor_name << " not found!";
    return false;
  }
  m_sensor_R_[sensor_name] = R;
  return true;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END