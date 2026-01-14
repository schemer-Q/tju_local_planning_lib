#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <any>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "tju_local_planning/algo/local_planning/QuinticPolynomialPlanner/local_trajectory_planner_base.h"
#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/pose_point.h"
#include "tju_local_planning/common/types/target_pose.h"
#include "tju_local_planning/common/types/odometry.h"
#include "tju_local_planning/common/types/local_trajectory_points.h"
#include "tju_local_planning/tools/log/t_log.h"

TJU_LOCAL_PLANNING_LIB_NAMESPACE_BEGIN
using namespace TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE;

// 局部规划器输入数据结构
struct LocalPlannerInput {
  PosePoint current_pose;          // 当前位姿
  PosePoint target_pose;          // 目标位姿
  std::vector<PosePoint> global_trajectory;  // 全局轨迹
  std::vector<PosePoint> obstacles;          // 障碍物
  double speed = 1.0;             // 期望速度
};

// 局部规划器输出数据结构
struct LocalPlannerOutput {
  LocalTrajectory trajectory;    // 规划的轨迹
  bool success = false;          // 规划是否成功
  std::string message;          // 状态信息
};

/**
 * @brief 五次多项式轨迹规划算法
 * 
 */
class QuinticPolynomialPlanner : public LocalTrajectoryPlannerBase {
 public:
  QuinticPolynomialPlanner() = default;
  ~QuinticPolynomialPlanner() = default;

  /**
   * @brief 初始化参数
   *
   * @param config yaml配置节点
   * @return int 0表示成功
   */
  int init(const YAML::Node& config) override;

  /**
   * @brief 处理路径规划
   *
   * @param input 规划输入数据
   * @return int 0表示成功
   */
  int process(const PosePoint& current_pose,const PosePoint& target_pose) override;

  /**
   * @brief 处理路径规划（重载以兼容基类接口）
   *
   * @return int 0表示成功
   */
  int process() override;

  /**
   * @brief 获取处理结果数据
   *
   * @param key 数据键名
   * @return std::any 返回的数据
   */
  std::any GetData(const std::string& key) override;


 private:
  // 输入参数
  PosePoint current_pose_;
  PosePoint target_pose_;
  std::vector<PosePoint> global_trajectory_;
  std::vector<PosePoint> obstacles_;
  
  std::vector<PosePoint> trajectory_;

  double step_length_ = 0.1;
  
  // 配置参数
  double default_speed_ = 1.0;  // 默认速度
  double max_accel, max_jerk, dt, MAX_T, MIN_T, spacing, output_tf_frame;

  // Internal parameters
  double sx, sy, syaw, sv, sa;
  double gx, gy, gyaw, gv, ga;
  
  // 输出结果
  LocalPlannerOutput output_;

  bool plan_success_ = false;

  // data_storage
  std::unordered_map<std::string, std::any> data_storage_;
  // internal functions
  struct Trajectory {
      std::vector<double> time;
      std::vector<double> x;
      std::vector<double> y;
      std::vector<double> yaw;
      std::vector<double> v;
      std::vector<double> a;
      std::vector<double> j;
      std::vector<double> curve;
    };
  struct ResampledPath {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> v;
    std::vector<double> a;
    std::vector<double> yaw;
    std::vector<double> curve;
    size_t num_points;
  };
  double normalize_angle(double angle);
  std::vector<double> datatransform(const PosePoint& current_pose, const PosePoint& target_pose); //transform for input data
  LocalPlannerOutput datatransform(const ResampledPath& path); //transform for output data
  Trajectory plan();
  ResampledPath resample_path(const std::vector<double>& rx, const std::vector<double>& ry, const std::vector<double>& rv, const std::vector<double>& ra, const std::vector<double>& ryaw, const std::vector<double>& rcurve);

  class QuinticPolynomial {
    public:
    QuinticPolynomial (double xs, double vxs, double axs, double xe, double vxe, double axe, double time);
    double calc_point(double t);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);
    private:
      double a0, a1, a2, a3, a4, a5;
  };
};

TJU_LOCAL_PLANNING_LIB_NAMESPACE_END
