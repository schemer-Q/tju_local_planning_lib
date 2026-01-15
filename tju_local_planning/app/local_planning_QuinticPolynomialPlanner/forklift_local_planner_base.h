#pragma once

#include <yaml-cpp/yaml.h>
#include <any>
#include <memory>
#include <string>
#include <vector>

#include "tju_local_planning/algo/local_planning/local_trajectory_planner_base.h"
#include "tju_local_planning/app/base/app_base.h"
#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/error/code.hpp"
#include "tju_local_planning/algo/local_planning/QuinticPolynomialPlanner/QuinticPolynomial/quintic_polynomial_planner.h"
#include "tju_local_planning/tools/log/t_log.h"

TJU_LOCAL_PLANNING_LIB_APP_NAMESPACE_BEGIN
using namespace TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE;

/**
 * @brief 叉车局部规划器基类
 */
class ForkLiftLocalPlannerBase : public AppBase {
 public:
  ForkLiftLocalPlannerBase();
  virtual ~ForkLiftLocalPlannerBase();

  /**
   * @brief 初始化应用
   *
   * @param config YAML配置
   * @return std::uint32_t 错误码
   */
  virtual std::uint32_t Init(const YAML::Node& config) override;

  /**
   * @brief 运行规划
   * @param cross_goal 全局规划终点
   * @param global_trajectory 全局轨迹
   * @param car_ori_data 车辆底盘数据
   * @param align_target 目标位姿
   * @param obstacles 障碍物数据
   * @param og_points 占用栅格点
   * @param fusion_location 融合定位
   * @return std::uint32_t 错误码
   */
  virtual std::uint32_t Run(
      const CrossGoal& cross_goal,
      const TrajectoryPoints& global_trajectory,
      const CarOriInterface& car_ori_data,
      const TargetPose& align_target,
      const Objects& obstacles,
      const OgmPoints& og_points,
      const Odometry& fusion_location) override;

  /**
   * @brief 获取数据
   *
   * @param key 数据键名
   * @return std::any 数据
   */
  virtual std::any GetData(const std::string& key) override;

 protected:

 // 碰撞检测参数结构体
struct CollisionCheckParams {
    double safety_distance = 0.5;      // 安全距离(米)
    double vehicle_width = 1.0;        // 车辆宽度(米)
    double vehicle_length = 2.0;       // 车辆长度(米)
    double obstacle_inflation = 0.3;   // 障碍物膨胀半径(米)
    bool check_dynamic_obstacles = true; // 是否检查动态障碍物
};

  // 碰撞检测参数
  CollisionCheckParams collision_params_;
  // 底层算法指针
  std::shared_ptr<LocalTrajectoryPlannerBase> planner_ptr_;
  std::shared_ptr<QuinticPolynomialPlanner> quintic_planner_ptr_;
  //局部路径
  //std::vector<PosePoint> path_data_;
  // 输出数据
  LocalTrajectory local_trajectory_;
  TrajectoryState trajectory_state_;
  std::vector<PosePoint> trajectory;

  //全局路径容器
  TrajectoryPoints global_trajectory;
  int64_t plan_counter_ = 0; // 规划计数器
  uint8_t task_type_ = 0; // 任务类型，0正常跟踪，1局部对准
  uint8_t plan_state_ = 0; // 规划状态，0未完成，1完成

  // 配置参数
  double off_track_threshold_;  // 偏离轨迹阈值
  double max_segment_length_;   // 最大分段长度
  size_t end_threshold_;       // 接近终点的阈值点数
  double min_speed_threshold_; // 最小速度阈值，用于计算匹配点
    bool use_global_tf = true;    // 是否使用全局坐标系
    double extension_distance = 2.0; // 两端规划的直线延申距离

  //重规划参数
  double tolarent_cte; //CTE误差容忍度 米
  double tolarent_heading; //heading误差容忍度 rad

  /**
   * @brief 弧度归一化
   */
  double normalize_angle(double angle) const{
      while (angle > M_PI) angle -= 2.0 * M_PI;
      while (angle < -M_PI) angle += 2.0 * M_PI;

      return angle;
  }
  /** 
   * @brief 执行规划
   * @param start_pose 起始位姿
   * @param end_pose 终止位姿
   * @return int suceess 或者 错误码
  */
  int plan(const PosePoint& start_pose, const PosePoint& end_pose, const PosePoint& mid_pose);

  /**
   * @brief 在轨迹中查找与当前位置最匹配的点
   * 
   * @param trajectory 轨迹点集
   * @param current_pose 当前位姿
   * @param nearest_idx 输出: 找到的最近点索引
   * @param consider_heading 是否考虑航向匹配
   * @return uint32_t 错误码
   */
  uint32_t findNearestPointIndex(
    const std::vector<PosePoint>& trajectory, 
    const PosePoint& current_pose,
    size_t& nearest_idx,
    bool consider_heading = false);

  /**
   * @brief 更新本地轨迹
   */
  void updateLocalTrajectory();

  /**
   * @brief 检查车辆是否接近全局轨迹终点
   * 
   * @param current_idx 当前在全局轨迹中的索引
   * @param global_trajectory 全局轨迹
   * @return bool 是否接近终点
   */
    bool isNearEndpoint(size_t current_idx, const std::vector<PosePoint>& global_trajectory);

  /**
   * @brief 更新状态
   */
  void updateTrajectoryState(
      double timestamp, 
      const Odometry& fusion_location,
      const std::vector<PosePoint>& trajectory,
      const PosePoint& current_pose);
  
  /**
   * @brief 检查跟踪误差是否超过容忍阈值
   * @param path 当前局部路径
   * @param tolerant 误差容忍度（CTE/heading）
   * @return bool 是否超出容忍度
   */
  bool checkCteAndHeading (const std::vector<PosePoint>& path, const PosePoint& current_pose, double& tolerant_cte, double& tolarent_heading);

  bool checkCollision(std::vector<PosePoint>& trajectory, const OgmPoints& og_points, const PosePoint& currentPose, double checkDistance = 10.0, double collisionThreshold = 3);
  std::vector<std::vector<PosePoint>> gear_process(const std::vector<PosePoint>& global_trajectory);

  void printf_red(const char *s)
  {
      printf("\033[0m\033[1;31m%s\033[0m", s);
  }

  void printf_yellow(const char *s)
  {
      printf("\033[0m\033[1;33m%s\033[0m", s);
  }

  void printf_green(const char *s)
  {
      printf("\033[0m\033[1;32m%s\033[0m", s);
  }

  /*调用a星的预处理，分两段进行规划，保证末尾为直线*/
  PosePoint preprocess(const PosePoint& end_pose);
};

TJU_LOCAL_PLANNING_LIB_APP_NAMESPACE_END
