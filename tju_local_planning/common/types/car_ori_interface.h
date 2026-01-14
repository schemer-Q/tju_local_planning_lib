#pragma once

#include <memory>

#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/header.h"
TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

struct CarOriInterface {
 public:
  // 对应std_msgs/Header header
  //uint64_t timestamp;
  Header header;
  
  double angle;           // 方向盘转角反馈
  double acc;             // 加速度
  double open_gas;        // 油门踏板开度
  int8_t drive_mode;      // 驾驶模式
  int8_t e_stop_trigger;  // 急停
  int8_t gear;            // 档位 
  double car_speed;       // 车速
  double motor_torque;    // 油门反馈
  int8_t yk_f;            // 复位H
  int8_t yk_h;            // 遥控F
  int8_t fault1;          // 故障代码1  
  int8_t fault2;          // 故障代码2  
  int8_t fault3;          // 故障代码3  
  int8_t fault4;          // 故障代码4  
  double mileage;         // 累计里程
  float brake_pressure;   // 制动压力采样值
  float lr_wheelspeed;    // 左后轮速
  float rr_wheelspeed;    // 右后轮速
  float soc;              // SOC
  int8_t carsts1;         // 车辆状态1
  int8_t carsts2;         // 车辆状态2
  double lx_hight;        // 高度传感器
  double pitching;        // 俯仰传感器
  int8_t carstartstate;   // 车辆启动状态
  float vcu_service_voltage; // VCU供电电压
  int32_t vcu_sts;        // VCU状态
  int8_t fault;           // 整车故障
  int8_t flag_end_wire;   // 线控反馈局部终点到达情况
  int8_t msg_from_wire_to_plan; // 线控到规划的消息

  typedef std::shared_ptr<CarOriInterface> Ptr;
  typedef std::shared_ptr<const CarOriInterface> ConstPtr;
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END
