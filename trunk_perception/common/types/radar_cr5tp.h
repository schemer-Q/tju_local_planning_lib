/**
 * @file radar_cr5tp.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief CR5TP雷达数据类型
 * @version 0.1
 * @date 2024-10-17
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

namespace cr5tp {

#define CR5TP_MAX_OBJECTS 12

enum struct RadarTrackStatus : uint8_t {
  TRACKED = 0,        // 跟踪
  PREDICTED = 1,      // 预测
  INVALID = 2,        // 无效
  SPARE1 = 3,         // 保留
  SPARE2 = 4,         // 保留
  SPARE3 = 5,         // 保留
  ERROR = 6,          // 错误
  NOT_AVAILABLE = 7,  // 无效
};

enum struct RadarObjectClass : uint8_t {
  OTHER = 0,           // 其他
  SPARE09 = 1,         // 保留
  WHEELER_4 = 2,       // 轮式车辆(4WHEELER)
  SPARE00 = 3,         // 保留
  WHEELER_2 = 4,       // 轮式车辆(2WHEELER)
  PEDESTRIAN = 5,      // 行人
  SPARE01 = 6,         // 保留
  SPARE02 = 7,         // 保留
  SPARE03 = 8,         // 保留
  SPARE04 = 9,         // 保留
  SPARE05 = 10,        // 保留
  SPARE06 = 11,        // 保留
  SPARE07 = 12,        // 保留
  SPARE08 = 13,        // 保留
  ERROR = 14,          // 错误
  NOT_AVAILABLE = 15,  // 无效
};

enum struct RadarObjectDynamicClass : uint8_t {
  RESERVED = 0,    // 保留
  STATIONARY = 1,  // 静止
  MOVABLE = 2,     // 移动
  ERROR = 3,       // 错误
};

enum struct RadarPrioritisation : uint8_t {
  FUNCTIONAL_PRIORITISATION = 0,     // 功能优先级
  NONFUNCTIONAL_PRIORITISATION = 1,  // 非功能优先级
  STATIC_PRIORITISATION = 2,         // 静态优先级
  NOT_AVAILABLE = 3,                 // 不可用
};

// 和DBC中SensorObjxx的信号一一对应, 按信号的开始位置排序
struct RadarObject {
  uint16_t crc;            // CRC 校验
  uint8_t counter;         // rolling counter
  float time_stamp;        // 基于时间同步信号给出的目标时间戳, ms
  float long_vel_var;      // 目标纵向速度方差
  float long_pos_var;      // 目标纵向位置方差
  float long_ext_fr_var;   // 目标纵向前后方方差
  float long_ext_bk_var;   // 目标纵向前后方方差
  float long_acc_var;      // 目标纵向加速度方差
  float lat_vel_var;       // 目标横向速度方差
  float lat_pos_var;       // 目标横向位置方差
  float lat_ext_rt_var;    // 目标横向右侧方差
  float lat_ext_left_var;  // 目标横向左侧方差
  float lat_acc_var;       // 目标横向加速度方差
  float heading_var;       // 目标航向方差
  float life_time;         // 目标生命周期, s
  float height;            // 目标高度, m
  float heading;           // 目标航向, rad, 目标前进方向与车辆前进方向的夹角, 逆时针为正
  uint8_t coast_index;     // 目标融合ID
  uint8_t obj_id;          // 目标 ID
  RadarObjectDynamicClass obj_dyn_class;  // 目标动态分类
  float obj_class_conf;                   // 目标分类置信度
  RadarObjectClass obj_class;             // 目标分类
  float long_acc;                         // 目标纵向加速度, m/s^2
  float lat_ext_right;                    // 目标横向右边界距离, m
  RadarTrackStatus track_status;          // 目标跟踪状态
  float long_vel;                         // 目标纵向速度, m/s
  float lat_vel;                          // 目标横向速度, m/s
  float lat_pos;                          // 目标横向位置, m, 左正右负
  float long_pos;                         // 目标纵向位置, m, 前正后负
  float long_ext_front;                   // 目标纵向前边界距离, m
  float long_ext_back;                    // 目标纵向后边界距离, m
  float lat_acc;                          // 目标横向加速度, m/s^2
  float lat_ext_left;                     // 目标横向左边界距离, m
  float exist_conf;                       // 目标存在置信度
  RadarPrioritisation prioritisation;     // 目标优先级
};

enum struct RadarFailureStatus : uint8_t {
  NO_FAILURE = 0,  // 无错误
  FAILURE = 1,     // 错误
  RESERVED = 2,    // 保留
  RESERVED2 = 3,   // 保留
};

enum struct RadarBlindnessStatus : uint8_t {
  NO_FAILURE = 0,  // 无错误
  BLINDNESS = 1,   // 盲区
  BLOCK = 2,       // 阻塞
  UNKNOWN = 3,     // 未知
};

enum struct RadarMainBoardStatus : uint8_t {
  NO_FAILURE = 0,          // 无错误
  UNDER_VOLTAGE = 1,       // 低压
  OVER_VOLTAGE = 2,        // 过压
  TEMPERATURE_DEFECT = 3,  // 温度异常
};

enum struct RadarMalfunctionStatus : uint8_t {
  NO_FAILURE = 0,             // 无错误
  FAILURE_IN_HORIZONTAL = 1,  // 水平方向故障
  FAILURE_IN_VERTICAL = 2,    // 垂直方向故障
  UNKOWN_STATUS = 3,          // 未知状态
};

struct RadarStatus {
  uint16_t crc;                            // CRC校验
  uint8_t counter;                         // 计数器
  RadarFailureStatus antenna_sts;          // 天线状态
  RadarBlindnessStatus blindness_sts;      // 盲区状态
  RadarFailureStatus dsp_sts;              // DSP状态
  RadarMainBoardStatus hw_main_board_sts;  // 主板状态
  RadarFailureStatus comm_input_sts;       // 通信输入状态
  RadarFailureStatus hw_mcu_sts;           // MCU状态
  RadarFailureStatus calib_sts;            // 校准状态
  RadarMalfunctionStatus mal_sts;          // 故障状态
};

struct RadarObjects {
  double timestamp;                        ///< 时间戳, 单位为秒
  uint64_t timestamp_ns;                   ///< 时间戳, ns
  RadarStatus radar_status;                ///< 雷达状态
  RadarObject objects[CR5TP_MAX_OBJECTS];  ///< 目标信息

  typedef std::shared_ptr<RadarObjects> Ptr;
  typedef std::shared_ptr<const RadarObjects> ConstPtr;
};

}  // namespace cr5tp

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END
