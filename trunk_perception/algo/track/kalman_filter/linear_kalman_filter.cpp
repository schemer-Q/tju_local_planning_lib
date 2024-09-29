/**
 * @file linear_kalman_filter.cpp
 * @author Fan Dongsheng
 * @brief 
 * @version 0.1
 * @date 2024-09-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "linear_kalman_filter.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

LinearKalmanFilter::LinearKalmanFilter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& H, const Eigen::MatrixXd& P,
                                       const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
    : A_(A), H_(H), P_(P), Q_(Q), R_(R), initialized_(false), X_(Eigen::VectorXd::Zero(A.rows())) {}

int LinearKalmanFilter::init(const Eigen::VectorXd& x0) {
  if (X_.size() != x0.size()) {
    TFATAL << "[LinearKalmanFilter] init state error!";
    return 1;
  }

  X_ = x0;
  initialized_ = true;
  return 0;
}

int LinearKalmanFilter::predict() {
  if (!initialized_) {
    TFATAL << "[LinearKalmanFilter] not initialized!";
    return 1;
  }

  X_ = A_ * X_;
  P_ = A_ * P_ * A_.transpose() + Q_;
  return 0;
}

int LinearKalmanFilter::update(const Eigen::VectorXd& z) {
  if (!initialized_) {
    TFATAL << "[LinearKalmanFilter] not initialized!";
    return 1;
  }

  const Eigen::MatrixXd K = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
  X_ += K * (z - H_ * X_);
  P_ = (Eigen::MatrixXd::Identity(A_.rows(), A_.rows()) - K * H_) * P_;
  return 0;
}

int LinearKalmanFilter::setState(const Eigen::VectorXd& X) {
  if (!initialized_) {
    TFATAL << "[LinearKalmanFilter] not initialized!";
    return 1;
  }

  if (X_.size() != X.size()) {
    TFATAL << "[LinearKalmanFilter] new state size error!";
    return 2;
  }

  X_ = X;
  return 0;
}

int LinearKalmanFilter::setStateTransitionMatrix(const Eigen::MatrixXd& A) {
  if (!initialized_) {
    TFATAL << "[LinearKalmanFilter] not initialized!";
    return 1;
  }

  if (A_.size() != A.size()) {
    TFATAL << "[LinearKalmanFilter] new state transition matrix size error!";
    return 2;
  }

  A_ = A;
  return 0;
}

int LinearKalmanFilter::setProcessNoiseMatrix(const Eigen::MatrixXd& Q) {
  if (!initialized_) {
    TFATAL << "[LinearKalmanFilter] not initialized!";
    return 1;
  }

  if (Q_.size() != Q.size()) {
    TFATAL << "[LinearKalmanFilter] new process noise matrix size error!";
    return 2;
  }

  Q_ = Q;
  return 0;
}

int LinearKalmanFilter::setMeasurementNoiseMatrix(const Eigen::MatrixXd& R) {
  if (!initialized_) {
    TFATAL << "[LinearKalmanFilter] not initialized!";
    return 1;
  }

  if (R_.size() != R.size()) {
    TFATAL << "[LinearKalmanFilter] new measurement noise matrix size error!";
    return 2;
  }

  R_ = R;
  return 0;
}

int LinearKalmanFilter::setStateCovarianceMatrix(const Eigen::MatrixXd& P) {
  if (!initialized_) {
    TFATAL << "[LinearKalmanFilter] not initialized!";
    return 1;
  }

  if (P_.size() != P.size()) {
    TFATAL << "[LinearKalmanFilter] new measurement noise matrix size error!";
    return 2;
  }

  P_ = P;
  return 0;
}

TRUNK_PERCEPTION_LIB_NAMESPACE_END