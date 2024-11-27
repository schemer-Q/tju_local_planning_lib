/**
 * @file radar_cubtektar.h
 * @author zzg
 * @brief 为彪角雷达数据类型
 * @version 0.1
 * @date 2024-11-25
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

namespace cubtektar{

#define CUBTEKTAR_MAX_OBJECTS 16

enum struct CubtekTarWarnPosType : uint8_t {
  RIGHT_WARNING = 0,  // 右边的警告信息
  LEFT_WARNING = 1,   // 左边的警告信息
  WARNING_NUM = 2,
};

enum struct CubtekTarRadarPosType : uint8_t {
  RIGHT_FRONT = 0,  // 右前雷达信息
  RIGHT_REAR = 1,   // 右后雷达信息
  LEFT_FRONT = 2,   // 左前雷达信息
  LEFT_REAR = 3,    // 左后雷达信息
  RADAR_NUM = 4,
};

enum struct CubtekTarCallIndexType : uint8_t {
  INDEX_INVALID = 0,
  INDEX_RIGHT_WARNING = 1,  // 右边的警告信息
  INDEX_LEFT_WARNING = 2,   // 左边的警告信息
  INDEX_RIGHT_FRONT = 3,    // 右前雷达信息
  INDEX_RIGHT_REAR = 4,     // 右后雷达信息
  INDEX_LEFT_FRONT = 5,     // 左前雷达信息
  INDEX_LEFT_REAR = 6,      // 左后雷达信息
};

enum struct CubtekTarDynStatType : uint8_t {
  UNCLASSIFIED = 0,  // 未分类
  STANDING = 1,      // 静止
  STOPPED = 2,       // 停止
  MOVING = 3,        // 移动
  ONCOMING = 4,      // 对向
  RESERVED0 = 5,     // 保留
  RESERVED1 = 6,     // 保留
  RESERVED2 = 7,     // 保留
};

enum struct CubtekTarMeasStatType : uint8_t {
  NEW = 0,        // 新目标
  PREDICTED = 1,  // 预测目标
  MEASURED = 2,   // 测量目标
  RESERVED = 3,   // 保留
};

enum struct CubtekTarCategoryType : uint8_t {
  UNDEFINE = 0,       // 未定义
  RESERVED0 = 1,      // 保留
  RESERVED1 = 2,      // 保留
  RESERVED2 = 3,      // 保留
  PEDESTRIAN = 4,     // 行人
  MOTORCYCLE = 5,     // 摩托车
  PASSENGER_CAR = 6,  // 小客车
  TRUCK = 7,          // 货车
};

enum struct CubtekTarRadarWarnLevelType : uint8_t {
  NO_WARNING = 0,            // 无警告
  FIRST_LEVEL_WARNING = 1,   // 一级警告
  SECOND_LEVEL_WARNING = 2,  // 二级警告
  ERROR = 3,                 // 错误
  SYSTEM_OFF = 4,            // 系统关闭
  SYSTEM_INITIAL = 5,        // 系统初始化
  RESERVED0 = 6,             // 保留
  RESERVED1 = 7,             // 保留
};

enum struct CubtekTarRadarWarnModeType : uint8_t {
  STANDBY = 0,      // 待机
  BSD = 1,          // 盲区检测, Blind Spot Detection
  LCA = 2,          // 换道辅助, Lane Change Assist
  RESERVED0 = 3,    // 保留
  RESERVED1 = 4,    // 保留
  RESERVED2 = 5,    // 保留
  RESERVED3 = 6,    // 保留
  RESERVED4 = 7,    // 保留
  RESERVED5 = 8,    // 保留
  RESERVED6 = 9,    // 保留
  RESERVED7 = 10,   // 保留
  TA = 11,          // 转弯辅助, Turn Assist
  RESERVED8 = 12,   // 保留
  RESERVED9 = 13,   // 保留
  RESERVED10 = 14,  // 保留
  RESERVED11 = 15,  // 保留
};

// obj 就是文档中的 target, 一个雷达目标, 注意这里的 x 是纵向
// 应该是右手坐标系, x 正方向是前方, y 正方向是左侧
struct RadarObject {
  float x;      // Longitudinal Position, m, 纵向位置
  float y;      // Lateral Position, m, 横向位置, 左正右负, 可实测一下
  float vx;     // Longitudinal Relative Velocity, m/s, 纵向相对速度
  float vy;     // Lateral Relative Velocity, m/s, 横向相对速度
  float acc_x;  // Longitudinal Relative Acceleration, m/s^2, 纵向相对加速度
  float cl;     // Confidence Level, 置信度, [0, 1]
  uint16_t id;                      // tracking number, ID
  CubtekTarDynStatType dyn_stat;    // Dynamic Status, 动态状态
  CubtekTarMeasStatType meas_stat;  // Measurement Status, 测量状态
  CubtekTarCategoryType category;   // Category, 类别
};

struct CubtekTarRadarWarningType {
  CubtekTarRadarWarnLevelType level;  // Warning Level, 警告级别
  CubtekTarRadarWarnModeType mode;    // Warning Mode, 警告模式
};

struct RadarObjects{
	double timestamp;                            // 时间戳, 单位为秒
	uint64_t timestamp_ns;                       // 时间戳, ns

	uint64_t timestamp_warn;                      // 警告信息时间戳, ns
	CubtekTarRadarWarningType warning;            // 警告信息
	uint64_t timestamp_radar;                     // 雷达信息时间戳, ns
	uint8_t obj_num;                              // 目标数量
	uint16_t frame_counter;                       // 帧计数器
	RadarObject objects[CUBTEKTAR_MAX_OBJECTS];   // 雷达目标信息

	typedef std::shared_ptr<RadarObjects> Ptr;
	typedef std::shared_ptr<const RadarObjects> ConstPtr;
};

}  // namespace cubtektar

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END



