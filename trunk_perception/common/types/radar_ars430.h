/**
 * @file radar.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 毫米波雷达数据类型
 * @version 0.1
 * @date 2024-10-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <cstdint>
#include <memory>
#include <vector>
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

namespace ars430 {

/**
 * @brief 雷达目标类型
 *
 */
enum struct RadarObjectType : uint8_t {
  POINT = 0,        ///< 点目标
  CAR = 1,          ///< 车辆
  TRUCK = 2,        ///< 卡车
  PEDESTRIAN = 3,   ///< 行人
  MOTORCYCLE = 4,   ///< 摩托车
  BICYCLE = 5,      ///< 自行车
  WIDE = 6,         ///< 宽目标
  UNCLASSIFIED = 7  // 未分类
};

/**
 * @brief 雷达目标运动状态
 *
 */
enum struct RadarObjectMoveStatus : uint8_t {
  UNKNOWN = 0,     ///< 未知
  MOVING = 1,      ///< 运动
  STOPPED = 2,     ///< 静止?
  STATIONARY = 3,  ///< 静止
  CROSSING = 4,    ///< 交叉, 横穿
  ONCOMING = 5,    ///< 对向
  ERR = 6,         ///< 错误
  SNA = 7          ///< 无效
};

/**
 * @brief 雷达目标隧道状态
 * 
 */
enum struct RadarObjectTunnelStatus : uint8_t {
  NOTDETECTED = 0,  ///< 未检测到
  DETECTED = 1,     ///< 检测到
  ERR = 2,          ///< 错误
  SNA = 3           ///< 无效
};

/**
 * @brief 雷达目标丢失原因
 * 
 */
enum struct RadarObjectLossReason : uint8_t {
  NO_REASON = 0,       ///< 无原因
  TARGET_LOST = 1,     ///< 目标丢失
  OBJ_DIST_FAR = 2,    ///< 目标距离太远
  OBJ_DIST_LEFT = 3,   ///< 目标距离靠左
  OBJ_DIST_RIGHT = 4,  ///< 目标距离靠右
  OBJ_LANE_LEST = 5,   ///< 目标在左车道
  OBJ_LANE_RIGHT = 6,  ///< 目标在右车道
  OBJ_CRV_LEFT = 7,    ///< 目标在左弯道
  OBJ_CRV_RIGHT = 8,   ///< 目标在右弯道
  LOSS_ERR = 14,       ///< 错误
  LOSS_SNA = 15        ///< 无效
};

/**
 * @brief 雷达目标维护状态
 * 
 */
enum struct RadarObjectMaintainStatus : uint8_t {
  DELETED = 0,      ///< 删除
  NEW = 1,          ///< 新建
  MEASURED = 2,     ///< 测量
  PREDICTED = 3,    ///< 预测
  INACTIVE = 4,     ///< 未激活
  MAXTYPEDIFF = 5,  ///< 最大类型差异
  ERR = 6,          // 错误
  SNA = 7           // 无效
};

/**
 * @brief 车体坐标系下的二维向量
 * @details x前 y左 z上, 原点在后轴中心?
 */
struct Vector2f {
  float x;  ///< x 坐标, 经度, longitude, 车辆前进方向为正
  float y;  ///< y 坐标, 纬度, latitude or lateral, 车辆左侧为正
};

/**
 * @brief 大陆ARSA430雷达数据类型
 *
 */
struct RadarObject {
  uint8_t id;                                 ///< 目标物识别号
  RadarObjectType type;                       ///< 目标物的分类，为 ACC 功能优化
  RadarObjectMoveStatus move_status;          ///< 目标物的(运动)类型
  RadarObjectTunnelStatus tunnel_status;      ///< 隧道观测状态现值
  RadarObjectLossReason loss_reason;          ///< 目标物消失原因
  RadarObjectMaintainStatus maintain_statue;  ///< 目标物的维护状态：猜测/测量的，创建原因，删除原因
  Vector2f distance2d_std;                    ///< 目标物的距离标准差
  Vector2f velocity2d_std;                    ///< 目标物的速度标准差
  uint16_t lifetime;                          ///< 从目标物创建开始的周期计数
  float rcs;                                  ///< 目标物雷达反射截面积(RCS)的现值 dBsm?
  float move_status_confidence;               ///< 估测的动态属性置信度  %
  float probability_of_existence;             ///< 目标物为真实目标的置信度  %
  float probability_of_obstacle;              ///< 目标物为障碍物的置信度  %
  Vector2f distance2d;                        ///< 目标距离  m
  Vector2f velocity2d;                        ///< 目标速度  m/s
  float acceleration_x;                       ///< 目标物纵向加速度  m/s^2
  float width;                                ///< 目标物的宽度  m
  float elevation_object_confidence;          ///< 高处目标物的置信度

  typedef std::shared_ptr<RadarObject> Ptr;
  typedef std::shared_ptr<const RadarObject> ConstPtr;
};

struct RadarObjects {
  double timestamp;       ///< 时间戳, 单位为秒
  uint64_t timestamp_ns;  ///< 时间戳, ns
  uint8_t obj_num = 0u;   ///< 目标物数量
  uint8_t func_status;    ///< 系统可用状态
  uint8_t blockage;       ///< 系统遮挡状态
  uint8_t busoff;         ///< CAN 总线状态

  std::vector<RadarObject> objects;

  typedef std::shared_ptr<RadarObjects> Ptr;
  typedef std::shared_ptr<const RadarObjects> ConstPtr;
};

}  // namespace ars430

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END