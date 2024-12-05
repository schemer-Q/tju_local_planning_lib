/**
 * @file center_tracker_cv.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief 
 * @version 0.1
 * @date 2024-12-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <Eigen/Core>

#include "trunk_perception/algo/track/tracker_method/center_tracker_cv.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

int CenterTrackerCV::Init(const YAML::Node& config, const Object& object) {
  // init kalman filter
  {
    constexpr int num_state = 4;
    constexpr int num_meas = 2;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_state, num_state);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(num_meas, num_state);
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(num_state, num_state);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(num_state, num_state);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(num_meas, num_meas);

    A.setIdentity();
    A(0, 2) = A(1, 3) = 0.1;
    H(0, 0) = H(1, 1) = 1.0;

    // 从调试结果看: 影响初始值收敛到真值的速度。
    P(0, 0) = 0.1;
    P(1, 1) = 0.1;
    P(2, 2) = 1650.0;
    P(3, 3) = 1650.0;
    P(0, 2) = P(2, 0) = 5.0;
    P(1, 3) = P(3, 1) = 5.0;

    // 从调试结果看: Q调大收敛速度快，但稳定性下降；Q调小收敛速度慢，但稳定性好
    Q(0, 0) = 0.15;
    Q(1, 1) = 0.15;
    Q(2, 2) = 0.35;
    Q(3, 3) = 0.35;

    // 从调试结果看: 影响收敛到真值附近的幅度（高低），R越大，越高；R越小，越低。
    R.setIdentity();
    R(0, 0) = R(1, 1) = 1.35;

    Eigen::Vector2d init_point = object.bbox.center.head(2).cast<double>();
    motion_filter_ = std::make_shared<LinearKalmanFilter>(A, H, P, Q, R);
    Eigen::VectorXd init_motion_state(num_state);
    init_motion_state << init_point, 0.0, 0.0;
    motion_filter_->init(init_motion_state);
  }

  initialized_ = true;
  return 0;
}

void CenterTrackerCV::Predict(const double dt, Object& object_tracked) {
  if (!initialized_) {
    TFATAL << "[CenterTrackerCV::Predict] not initialized!";
    return;
  }

  if (!motion_filter_) {
    TFATAL << "[CenterTrackerCV::Predict] motion_filter_ is nullptr!";
    return;
  }

  Eigen::MatrixXd A = motion_filter_->getStateTransitionMatrix();
  A(0, 2) = A(1, 3) = dt;
  motion_filter_->setStateTransitionMatrix(A);
  motion_filter_->predict();
  getTrackModel(object_tracked);
}

void CenterTrackerCV::Update(const Object& object, Object& object_tracked) {
  if (!initialized_) {
    TFATAL << "[CenterTrackerCV::Update] not initialized!";
    return;
  }

  if (!motion_filter_) {
    TFATAL << "[CenterTrackerCV::Update] motion_filter_ is nullptr!";
    return;
  }

  const Eigen::Vector2d measure_point = object.bbox.center.head(2).cast<double>();
  const double dt = object.timestamp - object_tracked.timestamp;
  Eigen::MatrixXd A = motion_filter_->getStateTransitionMatrix();
  A(0, 2) = A(1, 3) = dt;
  motion_filter_->setStateTransitionMatrix(A);
  motion_filter_->predict();
  motion_filter_->update(measure_point);

  getTrackModel(object_tracked);
}

void CenterTrackerCV::TransformToCurrent(const Eigen::Isometry3f& tf) {
  if (!initialized_) {
    TFATAL << "[CenterTrackerCV::TransformToCurrent] not initialized!";
    return;
  }

  if (!motion_filter_) {
    TFATAL << "[CenterTrackerCV::TransformToCurrent] motion_filter_ is nullptr!";
    return;
  }

  Eigen::VectorXd states_new = motion_filter_->getState();

  // transform tracking point
  const Eigen::Vector3d track_point(states_new(0), states_new(1), 0.0);
  states_new.head(2) = (tf.cast<double>() * track_point).head(2);

  // transform velocity
  const Eigen::Vector3d velocity(states_new(2), states_new(3), 0.0);
  states_new.segment(2, 2) = (tf.rotation().cast<double>() * velocity).head(2);

  // reset dynamic state
  motion_filter_->setState(states_new);
}

void CenterTrackerCV::getTrackModel(Object& object) {
  // track point
  object.track_point.head(2) = motion_filter_->getState().head(2).cast<float>();
  object.track_point(2) = 0.0F;

  // velocity
  object.velocity.head(2) = motion_filter_->getState().segment(2, 2).cast<float>();
  object.velocity(2) = 0.0F;

  // acceleration
  object.acceleration = Eigen::Vector3f::Zero();

  // state covariance matrix
  object.state_covariance = motion_filter_->getStateCovarianceMatrix().topLeftCorner(4, 4).cast<float>();
}

REGISTER_TRACKER_METHOD("CenterTrackerCV", CenterTrackerCV)

TRUNK_PERCEPTION_LIB_NAMESPACE_END