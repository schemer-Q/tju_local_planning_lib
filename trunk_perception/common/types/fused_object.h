/**
 * @file fused_object.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 后融合后的物体类
 * @version 0.1
 * @date 2024-10-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/object.h"
#include "trunk_perception/common/types/radar_ars430.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

/**
 * @brief 跟踪点类型
 *
 */
enum TrackPointType {
  Center = 0,
  RearMiddle = 1,
};

struct alignas(32) FusedObject {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int track_id = -1;       ///< 跟踪ID
  double timestamp = 0.0;  ///< 时间戳, s

  Eigen::Vector3f size = Eigen::Vector3f(0, 0, 0);  ///< 尺寸, [m]

  float theta = 0.0f;                                           ///< 偏航角, [rad]，local坐标系
  Eigen::Vector3d center = Eigen::Vector3d::Zero();             ///< 中心点, [m], local坐标系
  Eigen::Vector3d rear_middle_point = Eigen::Vector3d::Zero();  ///< 后边中心点, [m], local坐标系
  Eigen::Vector3d track_point = Eigen::Vector3d::Zero();        ///< 跟踪点, local坐标系
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();           ///< 物体速度, m/s，local坐标系
  Eigen::Vector3f acceleration = Eigen::Vector3f::Zero();       ///< 物体加速度, m/s^2，local坐标系

  float existence = 0.0f;                 ///< 存在概率
  float confidence = 0.0f;                ///< 置信度
  bool flag_special_keep_stable = false;  ///< 特殊情况保持稳定

  ObjectType type = ObjectType::UNKNOWN;  ///< 物体类型

  TrackPointType track_point_type = TrackPointType::Center;  ///< 跟踪点类型

  // 观测源信息
  int life = 0;                    ///< 生命周期, frame
  int lidar_total_life = 0;        ///< 激光雷达总生命周期, frame
  int front_radar_total_life = 0;  ///< 前向毫米波雷达总生命周期, frame

  int lidar_consecutive_hit = 0;   ///< 激光雷达连续命中帧数
  int lidar_consecutive_lost = 0;  ///< 激光雷达连续丢失帧数

	int lidar_consecutive_hit_his = 0;        ///< 激光雷达最近一次连续命中帧数             @author zzg 2024_12_04
	double lidar_consecutive_hit_his_ts = 0;  ///< 激光雷达最近一次连续命中帧数对应的时间戳    @author zzg 2024_12_04

  int front_radar_consecutive_hit = 0;   ///< 前向毫米波雷达连续命中帧数
  int front_radar_consecutive_lost = 0;  ///< 前向毫米波雷达连续丢失帧数

  
  std::vector<Eigen::Vector3d> GetConvexPoints() const {
    // 根据center, theta, size计算4个角点
    std::vector<Eigen::Vector3d> corners(4);

    // 计算旋转角的三角函数值
    double cos_yaw = std::cos(theta);
    double sin_yaw = std::sin(theta);

    // 计算半长和半宽
    double half_length = size.x() / 2.0;
    double half_width = size.y() / 2.0;

    // 计算四个角点（顺时针方向）
    // 右前角点
    corners[0] = center + Eigen::Vector3d(cos_yaw * half_length + sin_yaw * half_width,
                                          sin_yaw * half_length - cos_yaw * half_width, 0.0);

    // 右后角点
    corners[1] = center + Eigen::Vector3d(-cos_yaw * half_length + sin_yaw * half_width,
                                          -sin_yaw * half_length - cos_yaw * half_width, 0.0);

    // 左后角点
    corners[2] = center + Eigen::Vector3d(-cos_yaw * half_length - sin_yaw * half_width,
                                          -sin_yaw * half_length + cos_yaw * half_width, 0.0);

    // 左前角点
    corners[3] = center + Eigen::Vector3d(cos_yaw * half_length - sin_yaw * half_width,
                                          sin_yaw * half_length + cos_yaw * half_width, 0.0);

    return corners;
  }

  void InitTrackPoint() {
    switch (track_point_type) {
      case TrackPointType::Center:
        track_point = center;
        break;
      case TrackPointType::RearMiddle:
        track_point = rear_middle_point;
        break;
    }
  }

  void Transform(const Eigen::Matrix4d& trans_mat) {
    // 转换中心点
    Eigen::Vector4d center_vec(center.x(), center.y(), center.z(), 1.0);
    center_vec = trans_mat * center_vec;
    center = Eigen::Vector3d(center_vec.x(), center_vec.y(), center_vec.z());

    // 转换后中点
    Eigen::Vector4d rear_middle_point_vec(rear_middle_point.x(), rear_middle_point.y(), rear_middle_point.z(), 1.0);
    rear_middle_point_vec = trans_mat * rear_middle_point_vec;
    rear_middle_point =
        Eigen::Vector3d(rear_middle_point_vec.x(), rear_middle_point_vec.y(), rear_middle_point_vec.z());

    // 提取旋转矩阵(3x3)用于向量转换
    Eigen::Matrix3d rotation_matrix = trans_mat.block<3, 3>(0, 0);

    // 转换速度向量
    Eigen::Vector3d vel_vec(velocity.x(), velocity.y(), velocity.z());
    vel_vec = rotation_matrix * vel_vec;
    velocity = Eigen::Vector3f(vel_vec.x(), vel_vec.y(), vel_vec.z());

    // 转换加速度向量
    Eigen::Vector3d acc_vec(acceleration.x(), acceleration.y(), acceleration.z());
    acc_vec = rotation_matrix * acc_vec;
    acceleration = Eigen::Vector3f(acc_vec.x(), acc_vec.y(), acc_vec.z());

    // 转换偏航角
    double yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    theta += yaw;
    // 将角度归一化到[-π, π]
    theta = std::fmod(theta + M_PI, 2.0 * M_PI) - M_PI;

    InitTrackPoint();
  }

  typedef std::shared_ptr<FusedObject> Ptr;
  typedef std::shared_ptr<const FusedObject> ConstPtr;
};

enum class MeasureSensorType {
  Lidar = 0,
  ARS430Radar = 1,
};

struct alignas(32) SensorMeasureFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = 0.0;
  MeasureSensorType sensor_type = MeasureSensorType::Lidar;

  virtual ~SensorMeasureFrame() = default;

  /**
   * @brief 将坐标转换到另一个坐标系下
   *
   * @param trans_mat 转换矩阵
   */
  virtual void Transform(const Eigen::Matrix4d& trans_mat) = 0;

  typedef std::shared_ptr<SensorMeasureFrame> Ptr;
  typedef std::shared_ptr<const SensorMeasureFrame> ConstPtr;
};

struct alignas(32) LidarMeasureFrame : SensorMeasureFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MeasureSensorType sensor_type = MeasureSensorType::Lidar;

  DetectorType detector_type = DetectorType::UNKNOWN;  ///< 检测器类型

  float theta = 0.0f;                                           ///< 偏航角, [rad]
  Eigen::Vector3f size = Eigen::Vector3f(0, 0, 0);              ///< 尺寸, [m]
  Eigen::Vector3d center = Eigen::Vector3d::Zero();             ///< 中心点, [m]
  Eigen::Vector3d rear_middle_point = Eigen::Vector3d::Zero();  ///< 后边中心点, [m]
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();           ///< object velocity, m/s
  Eigen::Vector3f acceleration = Eigen::Vector3f::Zero();       ///< object acceleration

  Eigen::VectorXf type_probs;             ///< 物体类型概率
  float confidence = 0.0f;                ///< 置信度
  ObjectType type = ObjectType::UNKNOWN;  ///< 物体类型

  PointCloudPtr points_ptr = nullptr;                              ///< 物体点云指针
  Eigen::Matrix3Xf convex_polygon = Eigen::Matrix3Xf::Zero(3, 0);  ///< 凸包

  int lifetime = 0;
  int consecutive_lost = 0;
  size_t track_id = 0;

  Eigen::Matrix4f state_covariance = Eigen::Matrix4f::Zero();  ///< state covariance matrix

  LidarMeasureFrame(const Object& lidar_obj) {
    timestamp = lidar_obj.timestamp;
    sensor_type = MeasureSensorType::Lidar;

    detector_type = lidar_obj.detector_type;
    track_id = lidar_obj.track_id;
    theta = lidar_obj.bbox.theta;
    size = lidar_obj.bbox.size;
    center = Eigen::Vector3d(lidar_obj.bbox.center.x(), lidar_obj.bbox.center.y(), lidar_obj.bbox.center.z());
    type_probs = lidar_obj.type_probs;
    confidence = lidar_obj.confidence;
    type = lidar_obj.type;
    points_ptr = lidar_obj.points_ptr;
    convex_polygon = lidar_obj.convex_polygon;
    lifetime = 0;
    consecutive_lost = 0;
    velocity = lidar_obj.velocity;
    acceleration = lidar_obj.acceleration;
    state_covariance = lidar_obj.state_covariance;

    double cos_yaw = std::cos(theta);
    double sin_yaw = std::sin(theta);
    rear_middle_point = center + Eigen::Vector3d(-cos_yaw * size.x() / 2.0, -sin_yaw * size.x() / 2.0, 0.0);
  }

  /**
   * @brief 将坐标转换到另一个坐标系下
   *
   * @param trans_mat 转换矩阵
   */
  void Transform(const Eigen::Matrix4d& trans_mat) override {
    // 转换中心点
    Eigen::Vector4d center_vec(center.x(), center.y(), center.z(), 1.0);
    center_vec = trans_mat * center_vec;
    center = Eigen::Vector3d(center_vec.x(), center_vec.y(), center_vec.z());

    // 转换后中点
    Eigen::Vector4d rear_middle_point_vec(rear_middle_point.x(), rear_middle_point.y(), rear_middle_point.z(), 1.0);
    rear_middle_point_vec = trans_mat * rear_middle_point_vec;
    rear_middle_point =
        Eigen::Vector3d(rear_middle_point_vec.x(), rear_middle_point_vec.y(), rear_middle_point_vec.z());

    // 提取旋转矩阵(3x3)用于向量转换
    Eigen::Matrix3d rotation_matrix = trans_mat.block<3, 3>(0, 0);

    // 转换速度向量
    Eigen::Vector3d vel_vec(velocity.x(), velocity.y(), velocity.z());
    vel_vec = rotation_matrix * vel_vec;
    velocity = Eigen::Vector3f(vel_vec.x(), vel_vec.y(), vel_vec.z());

    // 转换加速度向量
    Eigen::Vector3d acc_vec(acceleration.x(), acceleration.y(), acceleration.z());
    acc_vec = rotation_matrix * acc_vec;
    acceleration = Eigen::Vector3f(acc_vec.x(), acc_vec.y(), acc_vec.z());

    // 转换偏航角
    double yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    theta += yaw;
    // 将角度归一化到[-π, π]
    theta = std::fmod(theta + M_PI, 2.0 * M_PI) - M_PI;
  }

  typedef std::shared_ptr<LidarMeasureFrame> Ptr;
  typedef std::shared_ptr<const LidarMeasureFrame> ConstPtr;
};

namespace ars430 {
struct alignas(32) RadarMeasureFrame : SensorMeasureFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RadarObject radar_obj;
  Eigen::Vector2d local_distance2d = Eigen::Vector2d::Zero();  ///< 局部坐标系下的位置
  Eigen::Vector2d local_velocity2d = Eigen::Vector2d::Zero();  ///< 局部坐标系下的速度

  RadarMeasureFrame(const RadarObject& radar_obj, const double& ts) : radar_obj(radar_obj) {
    timestamp = ts;
    sensor_type = MeasureSensorType::ARS430Radar;

    local_distance2d = Eigen::Vector2d(radar_obj.distance2d.x, radar_obj.distance2d.y);
    local_velocity2d = Eigen::Vector2d(radar_obj.velocity2d.x, radar_obj.velocity2d.y);
  }

  /**
   * @brief 将位置坐标转换到另一个坐标系下
   *
   * @param trans_mat 转换矩阵
   */
  void Transform(const Eigen::Matrix4d& trans_mat) override {
    // 转换位置坐标
    Eigen::Vector4d distance4d(radar_obj.distance2d.x, radar_obj.distance2d.y, 0, 1);
    distance4d = trans_mat * distance4d;
    local_distance2d = Eigen::Vector2d(distance4d.x(), distance4d.y());

    // 提取旋转矩阵用于速度向量变换
    Eigen::Matrix3d rotation_matrix = trans_mat.block<3, 3>(0, 0);

    // 转换速度向量
    Eigen::Vector3d velocity3d(radar_obj.velocity2d.x, radar_obj.velocity2d.y, 0);
    velocity3d = rotation_matrix * velocity3d;
    local_velocity2d = Eigen::Vector2d(velocity3d.x(), velocity3d.y());
  }

  typedef std::shared_ptr<RadarMeasureFrame> Ptr;
  typedef std::shared_ptr<const RadarMeasureFrame> ConstPtr;
};
}  // namespace ars430

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END