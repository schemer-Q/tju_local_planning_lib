#include "tju_local_planning/common/error/code.hpp"
#include <vector>
#include <cmath>
#include <iostream>
#include "tju_local_planning/app/local_planning_QuinticPolynomialPlanner/forklift_local_planner_base.h"
#include "tju_local_planning/algo/local_planning/local_trajectory_planner_manager.h"
#include "tju_local_planning/algo/local_planning/QuinticPolynomialPlanner/QuinticPolynomial/quintic_polynomial_planner.h"

TJU_LOCAL_PLANNING_LIB_APP_NAMESPACE_BEGIN

using namespace TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE;
ForkLiftLocalPlannerBase::ForkLiftLocalPlannerBase() = default;

ForkLiftLocalPlannerBase::~ForkLiftLocalPlannerBase() = default;

std::uint32_t ForkLiftLocalPlannerBase::Init(const YAML::Node& config) {
  try {

    // LOGINIT("./tju_local_planning.log", jian::logging::level_enum::trace, 5);
    // 检查配置有效性
    if (!config["Method"]) {
      NTERROR << "ForkLiftLocalPlannerBase::Init: Missing required configuration fields";
      return ErrorCode::PARAMETER_ERROR;
    }
    // LOGINIT("./log_demo.log", jian::logging::level_enum::trace, 5);
    // 从配置中获取规划器名称
    std::string planner_name = config["Method"].as<std::string>();
    
    // 使用LocalPlannerManager创建算法实例 返回五次多项式类的指针
    planner_ptr_ = LocalTrajectoryPlannerManager::Create(planner_name);
    if (!planner_ptr_) {
      NTERROR << "Failed to create local planner: " << planner_name;
      // std::cerr << "Failed to create local planner: " << planner_name << std::endl;
      return ErrorCode::PARAMETER_ERROR;
    }
    
    // 初始化算法
    int ret = planner_ptr_->init(config["Config"]);
    if (ret != 0) {
      NTERROR << "Failed to initialize local planner, error code: " << ret;
      return ErrorCode::UNINITIALIZED;
    }
    
    // 初始化local_trajectory_结构体
    local_trajectory_.header.stamp = 0;
    local_trajectory_.task_type = 0;  // 默认为正常跟踪
    local_trajectory_.points_cnt = 0;
    local_trajectory_.step_length = 0.0;
    local_trajectory_.replan_counter = 0;
    local_trajectory_.plan_state = 0;  // 默认未规划完成
    local_trajectory_.trajectory.clear();

    // 初始化trajectory_state_结构体
    trajectory_state_.header.stamp = 0;
    trajectory_state_.real_sensor_on_flag = 0;
    trajectory_state_.real_forklift_up_flag = 0;
    trajectory_state_.real_arrive_goal_pose = 0;
    trajectory_state_.idx_in_global = 0;
    
    // 读取步长
    if (config["Config"]["step_length"]) {
      local_trajectory_.step_length = config["Config"]["step_length"].as<double>();
    }
    
    // 读取偏离阈值
    if (config["Config"]["off_track_threshold_"]) {
      off_track_threshold_ = config["Config"]["off_track_threshold_"].as<double>();
    }
    
    // 读取最大分段长度
    if (config["Config"]["max_segment_length_"]) {
      max_segment_length_ = config["Config"]["max_segment_length_"].as<double>();
    }
    
    // 读取终点阈值
    if (config["Config"]["end_threshold_"]) {
      end_threshold_ = config["Config"]["end_threshold_"].as<size_t>();
    }

    // 读取最小速度阈值
    if (config["Config"]["min_speed_threshold_"]) {
      min_speed_threshold_ = config["Config"]["min_speed_threshold_"].as<double>();
    }

    if (config["Config"]["tolarent_cte"]) {
      tolarent_cte = config["Config"]["tolarent_cte"].as<double>();
    }

    if (config["Config"]["tolarent_heading"]) {
      tolarent_heading = config["Config"]["tolarent_heading"].as<double>();
    }

    if(config["Config"]["debug"]) {
      int debug_flag = config["debug"].as<int>();
      if (debug_flag == 1) {
        use_global_tf = true;
      } else {
        use_global_tf = false;
      }
    }
    
    std::cout << "ForkliftLocalPlannerBase initialized successfully with planner: " << planner_name << std::endl;
    
  } catch (const std::exception& e) {
    NTERROR << "ForkLiftLocalPlannerBase::Init failed: " << e.what();
    return ErrorCode::YAML_CONFIG_ERROR;
  }
  
  
  return ErrorCode::SUCCESS;
}

std::uint32_t ForkLiftLocalPlannerBase::Run(
  const CrossGoal& cross_goal,
  const TrajectoryPoints& global_trajectory_, //need test
  const CarOriInterface& car_ori_data,  //need test
  const TargetPose& align_target,  //need test
  const Objects& obstacles,
  const OgmPoints& og_points,
  const Odometry& fusion_location) { 

// NTINFO << "ForkLiftLocalPlannerBase::Run: start";
if (!planner_ptr_) {
  NTERROR << "ForkLiftLocalPlannerBase::Run: Planner not initialized";
  return ErrorCode::UNINITIALIZED;
}

NTTRACE << "ForkLiftLocalPlannerBase::Run: start";
printf_yellow("ForkLiftLocalPlannerBase::Run: start\n");
// 获取当前时间戳
double ts = fusion_location.time;

if (!global_trajectory.trajectory.empty()) printf_green("\n已接受到上游全局路径数据\n");
if (align_target.header.stamp != 0) printf_green("\n已接受到上游棉包位姿\n");

// 设置当前位姿（从融合定位获取）
PosePoint current_pose;
current_pose.x = fusion_location.position.x();
current_pose.y = fusion_location.position.y();
current_pose.z = fusion_location.position.z();
// 从四元数转换为欧拉角获取yaw
const auto& q = fusion_location.orientation;
current_pose.yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
current_pose.speed = car_ori_data.car_speed;
current_pose.gear = car_ori_data.gear;


//更新全局路径
if (!global_trajectory_.trajectory.empty()) {
  global_trajectory.trajectory = global_trajectory_.trajectory;
  global_trajectory.plan_state = global_trajectory_.plan_state; // 用于规定机械臂任务类型，指示调用a_star
  printf_yellow("全局路径已更新\n");
}

bool has_global_trajectory = !global_trajectory.trajectory.empty();
bool has_align_target = (has_cross_goal)&&((align_target.header.stamp != 0));

// 设置起点与终点
// std::cout<<"--------------1--------------"<<std::endl;
PosePoint start_pose = current_pose; 
PosePoint end_pose = align_target.align_target; //感知给出的车辆坐标系下棉包位姿

/*plan_state指示task_type任务类型，其值为1和5的时候，局部路径必须输出任务模式，否则输出智驾模式*/

/*根据任务类型选择是否调用a_star*/
if (global_trajectory.plan_state == 1 || global_trajectory.plan_state ==5) {
  if (align_target.header.stamp == 0) return ErrorCode::LOCAL_PLANNING_APP_USE_QUINTICPOLYNOMIALPLANNER;
  //调用一次a_star
  if (task_type_ == 1 && (!local_trajectory_.trajectory.empty())) return ErrorCode::SUCCESS;
  // 局部转全局坐标，如果需要的话
  if (!use_global_tf) {
    double target_x_global = current_pose.x + end_pose.x * cos(current_pose.yaw) - end_pose.y * sin(current_pose.yaw);
    double target_y_global = current_pose.y + end_pose.x * sin(current_pose.yaw) + end_pose.y * cos(current_pose.yaw);
    double target_yaw_global = normalize_angle(current_pose.yaw + end_pose.yaw);
    end_pose.x = target_x_global;
    end_pose.y = target_y_global;
    end_pose.yaw = target_yaw_global;
  }
  NTTRACE << "ForkLiftLocalPlannerBase::Run: Call a_star";
  printf_yellow("正在调用a_star\n");
  auto mid_pose = preprocess(end_pose);
  auto result = plan(start_pose, end_pose, mid_pose);
  updateLocalTrajectory();
  updateTrajectoryState(ts, fusion_location, trajectory, current_pose);
  return result;
}
/*非任务模式下翻译全局路径*/
else {
  if (global_trajectory.trajectory.empty()) return ErrorCode::LOCAL_PLANNING_EMPTY_TRAJECTORY;
  NTTRACE << "ForkLiftLocalPlannerBase::Run: Processing with global trajectory";
  printf_yellow("正在处理全局路径\n");
  trajectory = global_trajectory.trajectory;
  task_type_ = 0; 
  updateLocalTrajectory();
  updateTrajectoryState(ts, fusion_location, trajectory, current_pose);
  return ErrorCode::SUCCESS;
}
}
/*需要处理重规划和og点碰撞检测*/

// std::cout<<"-------------3---------------"<<std::endl;
/*先注释，但是后续需要考虑冲规划情况*/
// 需要重规划的情况
// if (has_align_target && checkCteAndHeading(trajectory, current_pose, tolarent_cte, tolarent_heading)) {
//   // 重规划
//   // std::cerr<<"forklift.cpp:: "<<"has_align_target && checkCteAndHeading"<<std::endl;
//   NTTRACE << "ForkLiftLocalPlannerBase::Run: Replan";
//   plan_counter_++;
//   auto result = plan(start_pose, end_pose);

// // 检测og点碰撞
//   if (!trajectory.empty()) {
//       if (checkCollision(trajectory, og_points, current_pose)) {
//           NTERROR << "Collision detected in planned trajectory!";
//       }
//   }

//   updateLocalTrajectory();
//   updateTrajectoryState(ts, fusion_location, has_global_trajectory, global_trajectory, current_pose);
//   return result;
// }

// 处理不同情况的规划
// std::cout<<"----------------4------------"<<std::endl;
// if (has_align_target)
// {
//   NTTRACE << "ForkLiftLocalPlannerBase::Run: Processing with align target";
//   auto result = plan(start_pose, end_pose);
//   plan_counter_ = 0; // 重置规划计数器

// // 检测og点碰撞
//   if (!trajectory.empty()) {
//       if (checkCollision(trajectory, og_points, current_pose)) {
//           NTERROR << "Collision detected in planned trajectory!";
//       }
//   }

//   updateLocalTrajectory();
//   updateTrajectoryState(ts, fusion_location, has_global_trajectory, global_trajectory, current_pose);
//   return result; // 调用plan函数进行规划
// } 


int ForkLiftLocalPlannerBase::plan (const PosePoint& start_pose, const PosePoint& end_pose, const PosePoint& mid_pose) {
    if (planner_ptr_) {
      /*第一段规划*/
      int ret1 = planner_ptr_->process(start_pose, mid_pose);
      if (ret1 != 0) {
        NTERROR << "ForkLiftLocalPlannerBase::Run: Failed to process planning"<< ret1;
        printf_red("第一段路径调用a星失败\n")
        local_trajectory_.plan_state = 0; // 规划失败
        return ErrorCode::LOCAL_PLANNING_APP_USE_QUINTICPOLYNOMIALPLANNER;
      }
      std::any now_data1 = planner_ptr_->GetData("trajectory");
      std::vector<PosePoint> trajectory1 = std::any_cast<std::vector<PosePoint>>(now_data1);
      /*第二段规划*/
      int ret2 = planner_ptr_->process(mid_pose, end_pose);
      if (ret2 != 0) {
        NTERROR << "ForkLiftLocalPlannerBase::Run: Failed to process planning"<< ret2;
        printf_red("第二段路径调用a星失败\n")
        local_trajectory_.plan_state = 0; // 规划失败
        return ErrorCode::LOCAL_PLANNING_APP_USE_QUINTICPOLYNOMIALPLANNER;
      }
      std::any now_data2 = planner_ptr_->GetData("trajectory");
      std::vector<PosePoint> trajectory2 = std::any_cast<std::vector<PosePoint>>(now_data2);
      /*路径拼接*/
      trajectory.clear();
      trajectory.insert(trajectory.end(), trajectory1.begin(), trajectory1.end());
      trajectory.insert(trajectory.end(), trajectory2.begin(), trajectory2.end());
      task_type_ = 1; 
      return ErrorCode::SUCCESS;
    }
}


std::any ForkLiftLocalPlannerBase::GetData(const std::string& key) {
  if (key == "local_trajectory") {
    if (local_trajectory_.task_type == 0) printf_green("当前输出为智驾模式\n");
    if (local_trajectory_.task_type == 1) printf_green("当前输出为任务模式\n");
    return local_trajectory_;
  } else if (key == "trajectory_state") {
    return trajectory_state_;
  }
  NTERROR << "ForkLiftLocalPlannerBase::GetData: No data available for key: " << key;
  
  return std::any();
}

uint32_t ForkLiftLocalPlannerBase::findNearestPointIndex(
  const std::vector<PosePoint>& trajectorys, 
  const PosePoint& current_pose,
  size_t& nearest_idx,
  bool consider_heading) {

if (trajectorys.empty()) {
  return ErrorCode::LOCAL_PLANNING_EMPTY_TRAJECTORY;
}


nearest_idx = 0;
double min_dist = std::numeric_limits<double>::max();

// 车辆速度向量
double vx = current_pose.speed * cos(current_pose.yaw);
double vy = current_pose.speed * sin(current_pose.yaw);
bool has_speed = std::abs(current_pose.speed) > min_speed_threshold_;

for (size_t i = 0; i < trajectorys.size(); ++i) {
  const auto& pt = trajectorys[i];
  double dist = std::hypot(pt.x - current_pose.x, pt.y - current_pose.y);
  
  if (dist < min_dist) {
    // 不考虑航向或车辆静止时，直接使用距离
    if (!consider_heading || !has_speed) {
      min_dist = dist;
      nearest_idx = i;
      if (nearest_idx >= trajectorys.size())return ErrorCode::LOCAL_PLANNING_INDEX_OUT_OF_BOUNDS;
    } else {
      // 考虑航向时，检查方向是否一致
      // 路径点的方向向量
      double px = cos(pt.yaw);
      double py = sin(pt.yaw);
      
      // 计算点积，判断方向是否一致
      double dot_product = vx * px + vy * py;
      
      // 如果方向相似（点积为正），则选为最近点
      if (dot_product > 0) {
        min_dist = dist;
        nearest_idx = i;
        if (nearest_idx >= trajectorys.size())return ErrorCode::LOCAL_PLANNING_INDEX_OUT_OF_BOUNDS;
      
      }
    }
  }
}

return ErrorCode::SUCCESS;
}

void ForkLiftLocalPlannerBase::updateLocalTrajectory() {
  // 更新轨迹元数据
  local_trajectory_.header.stamp = 0; // 微秒时间戳
  local_trajectory_.points_cnt = trajectory.size(); // 轨迹点数
  local_trajectory_.replan_counter = plan_counter_; // 规划计数器
  local_trajectory_.plan_state = plan_state_; // 规划状态
  local_trajectory_.task_type = task_type_;   // 设置任务类型 
  local_trajectory_.trajectory= trajectory; // 设置轨迹点
}

bool ForkLiftLocalPlannerBase::isNearEndpoint(size_t current_idx, const TrajectoryPoints& global_trajectory) {
  if (global_trajectory.trajectory.empty()) {
    return false;
  }
  
  // 使用配置的终点阈值判断是否接近终点
  return (current_idx >= global_trajectory.trajectory.size() - end_threshold_);
}

void ForkLiftLocalPlannerBase::updateTrajectoryState(
    double timestamp, 
    const Odometry& fusion_location,
    const std::vector<PosePoint>& trajectory,
    const PosePoint& current_pose) {
    
  // 初始化为空结构体
  trajectory_state_ = TrajectoryState();
  
  if (trajectory.empty()) {
    return;
  }
  
  // 查找当前位置在全局轨迹中的索引
  size_t nearest_idx;
  uint32_t ret = findNearestPointIndex(trajectory, current_pose, nearest_idx);
  if (ret == ErrorCode::SUCCESS) {
    trajectory_state_.idx_in_global = static_cast<uint8_t>(nearest_idx);
    trajectory_state_.real_time_point=fusion_location;
    
    // 检查是否接近终点并更新标志
    if (isNearEndpoint(nearest_idx, trajectory)) {
      trajectory_state_.real_sensor_on_flag = 1;
      trajectory_state_.real_forklift_up_flag = 1;
      trajectory_state_.real_arrive_goal_pose = 1;
    } else {
      trajectory_state_.real_sensor_on_flag = 0;
      trajectory_state_.real_forklift_up_flag = 0;
      trajectory_state_.real_arrive_goal_pose = 0;
    }
  }
}

PosePoint ForkLiftLocalPlannerBase::preprocess(const PosePoint& end_pose) {
  /* 1.计算两个候选中间点*/
  // 候选点1：沿货物朝向向前延伸，朝向与货物相同
    double mid_forward_x =  end_pose.x + extension_distance * std::cos(end_pose.yaw);
    double mid_forward_y = end_pose.y + extension_distance * std::sin(end_pose.yaw);
    double mid_forward_yaw = end_pose.yaw ;
  // 候选点2：沿货物朝向反方向向后延伸, 朝向与货物相同
    double mid_backward_x =  end_pose.x - extension_distance * std::cos(end_pose.yaw);
    double mid_backward_y = end_pose.y - extension_distance * std::sin(end_pose.yaw);
    double mid_backward_yaw = end_pose.yaw ;
  /* 2. 计算从候选点到目标点的期望朝向*/
  // 对于向前点
    double dx_forward = end_pose.x - mid_forward_x;
    double dy_forward = end_pose.y - mid_forward_y;
    double desired_yaw_forward = std::atan2(dy_forward, dx_forward);
  // 规范化角度差（考虑圆周性）
    double yaw_diff_forward = std::abs((desired_yaw_forward - mid_forward_yaw + M_PI) % (2 * M_PI) - M_PI);
  // 对于向后点
    double dx_backward = end_pose.x - mid_backward_x;
    double dy_backward = end_pose.y - mid_backward_y;
    double desired_yaw_backward = std::atan2(dx_backward, dy_backward);
    double yaw_diff_backward = std::abs((desired_yaw_backward - mid_backward_yaw + M_PI) % (2 * M_PI) - M_PI);
  /* 3. 判断哪个点的朝向与期望行驶方向一致*/
    double tol = 0.1;
  if (yaw_diff_forward < tol) {
        // 向前点满足条件：从该点到goal是直线，且车辆朝向就是行驶方向（正向驶入）
        PosePoint mid_pose;
        mid_pose.x = mid_forward_x;
        mid_pose.y = mid_forward_y;
        mid_pose.yaw = mid_forward_yaw;
        return mid_pose;
  } 
  if (yaw_diff_backward < tol) {
        // 向后点满足条件：从该点到goal是直线，且车辆朝向就是行驶方向（正向驶入）
        PosePoint mid_pose;
        mid_pose.x = mid_backward_x;
        mid_pose.y = mid_backward_y;
        mid_pose.yaw = mid_backward_yaw;
        return mid_pose;
  }
}

bool ForkLiftLocalPlannerBase::checkCteAndHeading (const std::vector<PosePoint>& path, const PosePoint& current_pose, double& tolarent_cte, double& tolarent_heading) {
  if (path.empty()) {return true;}
  size_t nearest_idx = 0;
  // double min_dist = std::numeric_limits<double>::max();
  // for (size_t i = 0; i < path.size(); ++i) {
  //     double dx = path[i].x - current_pose.x;
  //     double dy = path[i].y - current_pose.y;
  //     double dist = std::hypot(dx, dy);
  //     if (dist < min_dist) {
  //         min_dist = dist;
  //         nearest_idx = i;
  //     }
  // }
  uint32_t ret = findNearestPointIndex(path, current_pose, nearest_idx);
  const auto& ref = path[nearest_idx];
  double dx = current_pose.x - ref.x;
  double dy = current_pose.y - ref.y;
  double cte = std::abs(-std::sin(ref.yaw) * dx + std::cos(ref.yaw) * dy); // 取绝对值

  double heading_error = std::abs(current_pose.yaw - ref.yaw);
  // 归一化到 [0, pi]
  if (heading_error > M_PI) heading_error = 2 * M_PI - heading_error;

  // 判断是否超出阈值
  if ( cte<tolarent_cte && heading_error<tolarent_heading ) {return false;}
  else {return true;}
}

bool ForkLiftLocalPlannerBase::checkCollision (std::vector<PosePoint>& trajectory, const OgmPoints& og_points, const PosePoint& currentPose, double checkDistance, double collisionThreshold) {
    
  if (trajectory.empty() || og_points.points.empty()) {
        return false;
    }
  std::cout<<"CHECKING::::"<<std::endl;
    // 1. 找到局部路径中距离当前定位点最近的点索引
    size_t nearestIndex = 0;
    double minDist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < trajectory.size(); ++i) {
        double dx = trajectory[i].x - currentPose.x;
        double dy = trajectory[i].y - currentPose.y;
        double dist = std::hypot(dx, dy);
        
        if (dist < minDist) {
            minDist = dist;
            nearestIndex = i;
        }
    }

    // 2. 从最近点开始遍历局部路径，最多检查checkDistance距离
    double accumulatedDistance = 0.0;
    bool collisionDetected = false;
    size_t collisionStartIndex = trajectory.size();  // 初始化为无效值
    
    for (size_t i = nearestIndex; i < trajectory.size(); ++i) {
        // 计算累计距离（从最近点到当前点的路径长度）
        if (i > nearestIndex) {
            double dx = trajectory[i].x - trajectory[i-1].x;
            double dy = trajectory[i].y - trajectory[i-1].y;
            accumulatedDistance += std::hypot(dx, dy);
        }
        
        // 如果累计距离超过检查范围，停止遍历
        if (accumulatedDistance > checkDistance) {
            break;
        }
        
        // 检查当前路径点是否与任何障碍物碰撞
        for (const auto& obstacle : og_points.points) {
            double dx = trajectory[i].x - obstacle.x;
            double dy = trajectory[i].y - obstacle.y;
            double dist = std::hypot(dx, dy);
            // std::cout<<"og_dist::"<<dist<<std::endl;
            if (dist < collisionThreshold) {
                collisionDetected = true;
                collisionStartIndex = i;
                std::cout<<"ind::"<<i+nearestIndex<<std::endl;
                break;
            }
        }
        
        if (collisionDetected) {
            break;
        }
    }

  
    if (collisionDetected && collisionStartIndex > nearestIndex) {
        double start_speed = trajectory[nearestIndex].speed;
        size_t decel_len = collisionStartIndex - nearestIndex;
        for (size_t i = nearestIndex; i <= collisionStartIndex; ++i) {
            double ratio = static_cast<double>(i - nearestIndex) / decel_len;
            trajectory[i].speed = start_speed * (1.0 - ratio); // 线性递减
        }
        // 后续点速度仍设为0
        for (size_t i = collisionStartIndex + 1; i < trajectory.size(); ++i) {
            if (i > nearestIndex) {
                double dx = trajectory[i].x - trajectory[i-1].x;
                double dy = trajectory[i].y - trajectory[i-1].y;
                accumulatedDistance += std::hypot(dx, dy);
                if (accumulatedDistance > checkDistance) {
                    break;
                }
            }
            trajectory[i].speed = 0.0;
        }
    }
    return collisionDetected;
  }

TJU_LOCAL_PLANNING_LIB_APP_NAMESPACE_END
