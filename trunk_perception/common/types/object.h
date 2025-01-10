/**
 * @file object.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 适用于物体检测结果
 * @version 0.1
 * @date 2024-09-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "Eigen/src/Core/Matrix.h"

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/point.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

/**
 * @brief 物体类型
 *
 */
enum class ObjectType : std::uint8_t {
  UNKNOWN = 0,  ///< 未知; rule-based 方法结果
  /********************************************************/
  VEHICLE = 1,     ///< 车
  BICYCLE = 2,     ///< 自行车、摩托车
  PEDESTRIAN = 3,  ///< 行人
  BUS = 4,         ///< 大巴车
  TRICYCLE = 5,    ///< 三轮车
  CONE = 6,        ///< 锥桶
  BARREL = 7,      ///< 桶
  /********************************************************/
  ART = 10,               ///< ART (通用类型)
  ART_NO_TRAILER = 11,    ///< 不带箱ART
  ART_SEMI_TRAILER = 12,  ///< 带20吋箱ART
  ART_FULL_TRAILER = 13,  ///< 带40吋箱ART
  /********************************************************/
  TRUCK = 20,               ///< 卡车 (通用类型)
  TRUCK_HEAD = 21,          ///< 卡车头
  TRAILER = 22,             ///< 挂车
  TRUCK_NO_TRAILER = 23,    ///< 不带箱挂车
  TRUCK_SEMI_TRAILER = 24,  ///< 带20吋箱挂车
  TRUCK_FULL_TRAILER = 25,  ///< 带40吋箱挂车
  /********************************************************/
  ALIEN_VEHICLE = 30,         ///< 异形车 (通用类型)
  FORKLIFT = 31,              ///< 叉车
  ECCENTRIC_TRUNK_HEAD = 32,  ///< 偏头车
  MOBILE_CRANE = 33,          ///< 流机
  CRANE_LEG = 34,             ///< 桥腿
  /********************************************************/
  OTHERS = 40,  ///< 其他
  /********************************************************/
  SIZE = 24,  ///< ObjectType 枚举类成员数量(不包含SIZE类型本身)
};

static std::unordered_map<ObjectType, std::string> ObjectTypeDict = {
    {ObjectType::UNKNOWN, "UNKNOWN"},
    /********************************************************/
    {ObjectType::VEHICLE, "VEHICLE"},
    {ObjectType::BICYCLE, "BICYCLE"},
    {ObjectType::PEDESTRIAN, "PEDESTRIAN"},
    {ObjectType::BUS, "BUS"},
    {ObjectType::TRICYCLE, "TRICYCLE"},
    /********************************************************/
    {ObjectType::ART, "ART"},
    {ObjectType::ART_NO_TRAILER, "ART_NO_TRAILER"},
    {ObjectType::ART_SEMI_TRAILER, "ART_SEMI_TRAILER"},
    {ObjectType::ART_FULL_TRAILER, "ART_FULL_TRAILER"},
    /********************************************************/
    {ObjectType::TRUCK, "TRUCK"},
    {ObjectType::TRUCK_HEAD, "TRUCK_HEAD"},
    {ObjectType::TRAILER, "TRAILER"},
    {ObjectType::TRUCK_NO_TRAILER, "TRUCK_NO_TRAILER"},
    {ObjectType::TRUCK_SEMI_TRAILER, "TRUCK_SEMI_TRAILER"},
    {ObjectType::TRUCK_FULL_TRAILER, "TRUCK_FULL_TRAILER"},
    /********************************************************/
    {ObjectType::ALIEN_VEHICLE, "ALIEN_VEHICLE"},
    {ObjectType::FORKLIFT, "FORKLIFT"},
    {ObjectType::ECCENTRIC_TRUNK_HEAD, "ECCENTRIC_TRUNK_HEAD"},
    {ObjectType::MOBILE_CRANE, "MOBILE_CRANE"},
    {ObjectType::CRANE_LEG, "CRANE_LEG"},
};

/**
 * @brief 检测器类型
 *
 */
enum class DetectorType {
  UNKNOWN = 0,
  LidarModel = 1,
  VisionModel = 2,
  RuleBased = 3,
};

/**
 * @brief 检测器类型字典
 *
 */
static std::unordered_map<DetectorType, std::string> DetectorTypeDict = {
    {DetectorType::LidarModel, "LidarModel"},
    {DetectorType::VisionModel, "VisionModel"},
    {DetectorType::RuleBased, "RuleBased"},
};

/**
 * @brief L形特征类
 *
 */
struct alignas(16) LShapeFeature {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d center_point = Eigen::Vector2d::Zero();     ///< 中心点
  Eigen::Vector2d reference_point = Eigen::Vector2d::Zero();  ///< 最近角点
  Eigen::Vector3d shape = Eigen::Vector3d::Zero();            ///< L1、L2、theta_L1
};

/**
 * @brief 尾边中点特征
 *
 */
struct TailCenterFeature {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 2, 4> edge_center_points = Eigen::Matrix<double, 2, 4>::Zero();  // 各边的中点(从尾边顺时针排列)
  Eigen::Vector2d tail_center_point = Eigen::Vector2d::Zero();                           // 尾边中心点
};

/**
 * @brief 检测框类
 *
 */
struct BoundingBox {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3f direction = Eigen::Vector3f(1, 0, 0);  ///< 主方向, [x,y,z], 默认[1, 0, 0]表示x轴
  float theta = 0.0f;                                    ///< 偏航角, [rad]
  Eigen::Vector3f center = Eigen::Vector3f(0, 0, 0);     ///< 中心点, [m]
  Eigen::Vector3f size = Eigen::Vector3f(0, 0, 0);       ///< 尺寸, [m]
  Eigen::Matrix<float, 2, 4> corners2d = Eigen::Matrix<float, 2, 4>::Zero();  ///< 2D 角点, [x,y]

  BoundingBox() = default;
  BoundingBox(float x_, float y_, float z_, float l_, float w_, float h_, float rt_)
      : theta(rt_), center({x_, y_, z_}), size(l_, w_, h_) {}
};

/**
 * @brief 物体类
 *
 */
struct alignas(32) Object {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int detect_id = -1;                                  ///< 检测器给出的物体ID，给不了的设置为-1
  double timestamp = 0.0;                              ///< 时间戳，单位为秒
  DetectorType detector_type = DetectorType::UNKNOWN;  ///< 检测器类型
  BoundingBox bbox;                                    ///< 物体检测框
  Eigen::VectorXf type_probs;                          ///< 物体类型概率
  float confidence = 0.0f;                             ///< 置信度
  ObjectType type = ObjectType::UNKNOWN;               ///< 物体类型

  PointCloudPtr points_ptr = nullptr;  ///< 物体点云指针

  Eigen::Matrix3Xf convex_polygon = Eigen::Matrix3Xf::Zero(3, 0);  ///< 凸包

  LShapeFeature l_shape_feature;  ///< L形特征

  TailCenterFeature tail_center_feature;  ///< 尾边中心点特征

  // track param
  size_t track_id = 0UL;                                       ///< track id
  int lifetime = 0;                                            ///< track lifetime
  int consecutive_lost = 0;                                    ///< consecutive lost number
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();          ///< object velocity, m/s
  Eigen::Vector3f acceleration = Eigen::Vector3f::Zero();      ///< object acceleration
  Eigen::Vector3f track_point = Eigen::Vector3f::Zero();       ///< track point
  Eigen::Matrix4f state_covariance = Eigen::Matrix4f::Zero();  ///< state covariance matrix
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END