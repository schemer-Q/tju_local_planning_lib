/**
 * @file linear_kalman_filter.h
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

class LinearKalmanFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LinearKalmanFilter() = delete;
  ~LinearKalmanFilter() = default;
  LinearKalmanFilter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& H, const Eigen::MatrixXd& P,
                     const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);

 public:
  /**
   * @brief Init Kalman State
   *
   * @param x0 Init State param
   * @return int
   *    @retval 0 normal
   *    @retval 1 state vertor size not correct
   */
  int init(const Eigen::VectorXd& x0);

  /**
   * @brief Kalman State predict
   *
   * @return int
   *    @retval 0 normal
   *    @retval 1 kalman not initialized
   */
  int predict();

  /**
   * @brief Kalman State update
   *
   * @param z Measurement vector
   * @return int
   *    @retval 0 normal
   *    @retval 1 kalman not initialized
   */
  int update(const Eigen::VectorXd& z);

  /**
   * @brief Set the State vertor
   *
   * @param state New State vertor
   * @return int
   *    @retval 0 normal
   *    @retval 1 kalman not initialized
   *    @retval state vertor size not same
   */
  int setState(const Eigen::VectorXd& X);

  /**
   * @brief Set the State Transition Matrix
   *
   * @param A New State Transition Matrix
   * @return int
   *    @retval 0 normal
   *    @retval 1 kalman not initialized
   *    @retval 2 State Transition Matrix size not same
   */
  int setStateTransitionMatrix(const Eigen::MatrixXd& A);

  /**
   * @brief Set the Process Noise Matrix
   *
   * @param Q New Process Noise Matrix
   * @return int
   *    @retval 0 normal
   *    @retval 1 kalman not initialized
   *    @retval 2 Process Noise Matrix size not same
   */
  int setProcessNoiseMatrix(const Eigen::MatrixXd& Q);

  /**
   * @brief Set the Measurement Noise Matrix
   *
   * @param R New Measurement Noise Matrix
   * @return int
   *    @retval 0 normal
   *    @retval 1 kalman not initialized
   *    @retval 2 Measurement Noise Matrix size not same
   */
  int setMeasurementNoiseMatrix(const Eigen::MatrixXd& R);

  /**
   * @brief Set the State Covariance Matrix
   *
   * @param P
   * @return int
   */
  int setStateCovarianceMatrix(const Eigen::MatrixXd& P);

  /**
   * @brief Get the State vertor
   *
   * @return Eigen::VectorXd
   */
  Eigen::VectorXd getState() const { return X_; };

  /**
   * @brief Get the State Transition Matrix
   *
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd getStateTransitionMatrix() const { return A_; };

  /**
   * @brief Get the Process Noise Matrix
   *
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd getProcessNoiseMatrix() const { return Q_; };

  /**
   * @brief Get the Measurement Noise Matrix
   *
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd getMeasurementNoiseMatrix() const { return R_; };

  /**
   * @brief Get the State Covariance Matrix
   *
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd getStateCovarianceMatrix() const { return P_; };

 private:
  bool initialized_ = false;
  Eigen::VectorXd X_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END