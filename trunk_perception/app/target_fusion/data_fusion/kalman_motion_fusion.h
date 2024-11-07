/**
 * @file motion_fusion.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 多观测的Kalman Filter
 * @version 0.1
 * @date 2024-10-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <Eigen/Dense>
#include <unordered_map>

#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 运动模型枚举类
 *
 */
enum MotionModel {
  CV,  ///< 状态空间 [x,y,vx,vy], 匀速直线模型
};

[[maybe_unused]] static MotionModel GetMotionModel(const std::string& motion_model) {
  if (motion_model == "CV") {
    return MotionModel::CV;
  }
  return MotionModel::CV;
}

struct MotionFusionConfig {
  MotionModel motion_model = MotionModel::CV;                 ///< 运动模型
  std::unordered_map<std::string, Eigen::MatrixXd> sensor_R;  ///< 不同观测源的测量噪声矩阵
  std::unordered_map<std::string, Eigen::MatrixXd> sensor_H;  ///< 不同观测源的观测矩阵
};

/**
 * @brief 多观测的Kalman Filter
 * @details 支持来自Lidar、前毫米波雷达、角毫米波雷达、视觉BEV检测的多个观测结果对物体运动状态进行更新
 */
class KalmanMotionFusion {
 public:
  /**
   * @brief Construct a new Kalman Motion Fusion object
   *
   * @param config [IN] 配置
   */
  explicit KalmanMotionFusion(const MotionFusionConfig& config);
  ~KalmanMotionFusion();

  /**
   * @brief 初始化Kalman Filter
   *
   * @param x0 初始状态向量
   * @return true 初始化成功
   * @return false 初始化失败
   */
  bool Init(const Eigen::VectorXd& x0);

  /**
   * @brief 预测
   *
   * @param dt [IN] 时间间隔, 单位秒
   * @return true 预测成功
   * @return false 预测失败
   */
  bool Predict(const double& dt);

  /**
   * @brief 更新
   *
   * @param sensor_name [IN] 观测源名称
   * @param z [IN] 观测值
   * @return true 更新成功
   * @return false 更新失败
   */
  bool Update(const std::string& sensor_name, const Eigen::VectorXd& z);

  /**
   * @brief 获取状态向量
   *
   * @return Eigen::VectorXd 状态向量
   */
  Eigen::VectorXd GetState() const;

  bool SetSensorR(const std::string& sensor_name, const Eigen::MatrixXd& R);

 private:
  bool initialized_ = false;                                     ///< 是否初始化
  MotionModel motion_model_;                                     ///< 运动模型
  Eigen::VectorXd X_;                                            ///< 状态向量
  Eigen::MatrixXd A_;                                            ///< 状态转移矩阵
  Eigen::MatrixXd P_;                                            ///< 状态协方差矩阵
  Eigen::MatrixXd Q_;                                            ///< 过程噪声矩阵
  std::unordered_map<std::string, Eigen::MatrixXd> m_sensor_R_;  ///< 不同观测源的测量噪声矩阵
  std::unordered_map<std::string, Eigen::MatrixXd> m_sensor_H_;  ///< 不同观测源的观测矩阵
  double default_dt_ = 0.1;                                      ///< 默认时间间隔, 单位秒
};                                                               // namespace KalmanMotionFusionclass KalmanMotionFusion

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END