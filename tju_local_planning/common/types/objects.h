#pragma once

#include <vector>
#include <memory>

#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/header.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

struct Point {
 public:
  double x;
  double y;
  double z;

  typedef std::shared_ptr<Point> Ptr;
  typedef std::shared_ptr<const Point> ConstPtr;
};

struct Object {
 public:
  //double timestamp;           // 时间戳
  Header header;              // 对应std_msgs/Header header
  uint16_t sensor_id;         // 传感器id
  uint8_t id;                 // 障碍物id
  uint8_t tracking_status;    // 追踪状态
  uint8_t classification;     // 分类
  uint8_t moving_status;      // 运动状态
  uint16_t age;               // 生命周期
  float exist_confidence;     // 存在置信度
  float heading;              // 航向角
  float heading_cov;          // 航向角的协方差
  float length;               // 障碍物长度
  float width;                // 障碍物宽度
  float height;               // 障碍物高度
  float length_cov;           // 障碍物长度协方差
  float width_cov;            // 障碍物宽度协方差
  float height_cov;           // 障碍物高度协方差
  float dx;                   // X方向上偏移吸收
  float dy;                   // y方向上跳动吸收
  float abs_vx;               // x方向速度
  float abs_vy;               // y方向速度
  float dx_cov;               // Dx协方差
  float dy_cov;               // Dy协方差
  float vx_cov;               // x方向速度协方差
  float vy_cov;               // y方向速度协方差
  float ax;                   // x方向加速度
  float ay;                   // y方向加速度
  float ax_cov;               // x方向加速度协方差
  float ay_cov;               // y方向加速度协方差
  uint8_t light;              // 障碍物高度
  std::vector<Point> contour_points; // 障碍物角点

  typedef std::shared_ptr<Object> Ptr;
  typedef std::shared_ptr<const Object> ConstPtr;
};

struct Objects {
 public:
  Header header;         // 对应std_msgs/Header header
  std::vector<Object> objects; // object list

  typedef std::shared_ptr<Objects> Ptr;
  typedef std::shared_ptr<const Objects> ConstPtr;
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END
