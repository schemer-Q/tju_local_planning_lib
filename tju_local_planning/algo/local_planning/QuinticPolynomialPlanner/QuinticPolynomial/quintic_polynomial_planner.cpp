#include "tju_local_planning/algo/local_planning/QuinticPolynomialPlanner/QuinticPolynomial/quintic_polynomial_planner.h"
#include <cmath>
#include <iostream>

TJU_LOCAL_PLANNING_LIB_NAMESPACE_BEGIN

using SizeType = std::size_t;

int QuinticPolynomialPlanner::init(const YAML::Node& config) {
  try {
    default_speed_ = config["default_speed"].as<double>();
    MAX_T = config["MAX_T"].as<double>();
    MIN_T = config["MIN_T"].as<double>();
    max_accel = config["max_accel"].as<double>();
    max_jerk = config["max_jerk"].as<double>();
    dt = config["dt"].as<double>();
    spacing = config["spacing"].as<double>();
    output_tf_frame = config["output_tf_frame"].as<int>();
  }
  catch (YAML::InvalidNode e) {
    NTWARNING << "load config failed and use default value" ;
    //using default value
    MAX_T = 100.0;
    MIN_T = 5.0;
    max_accel = 1.0;
    max_jerk = 0.5;
    dt = 0.1;
    spacing = 0.2;
    default_speed_ = 1.0;
    output_tf_frame = 0;
  }
  return 0;
}

int QuinticPolynomialPlanner::process() {
  return process(current_pose_, target_pose_);
}

int QuinticPolynomialPlanner::process(const PosePoint& current_pose,const PosePoint& target_pose) {
  current_pose_ = current_pose;
  target_pose_ = target_pose;
  // test
  //std::cout<<"start point"<<current_pose.x<<" "<<current_pose.y<<std::endl;
  //std::cout<<"target point"<<target_pose.x<<" "<<target_pose.y<<std::endl;
  auto param = datatransform(current_pose, target_pose);
  sx = param[0];
  sy = param[1];
  syaw = param[2];
  sv = param[3];
  sa = param[4];
  gx = param[5];
  gy = param[6];
  gyaw = param[7];
  gv = default_speed_;
  ga = 0.0;
  Trajectory traj = plan();
  ResampledPath resampled_path = resample_path(traj.x, traj.y, traj.v, traj.a, traj.yaw, traj.curve);
  LocalPlannerOutput output = datatransform(resampled_path);
  trajectory_.clear();
  trajectory_ = output.trajectory.trajectory;
  // data_storage_.clear();
  // data_storage_["output"] = output;
  // data_storage_["trajectory"] = output.trajectory.trajectory;
  // data_storage_["success"] = output.success;
  // data_storage_["message"] = output.message;
  return 0;
}

std::any QuinticPolynomialPlanner::GetData(const std::string& key) {
  // NTWARNING << "错误的调用方式，请使用对象指针访问数据" ;
  
  return trajectory_;
}

QuinticPolynomialPlanner::Trajectory QuinticPolynomialPlanner::plan() {
  // Calculate start and end velocities in x and y
  double vxs = sv * cos(syaw);
  double vys = sv * sin(syaw);
  double vxg = gv * cos(gyaw);
  double vyg = gv * sin(gyaw);

  // Calculate start and end accelerations in x and y
  double axs = sa * cos(syaw);
  double ays = sa * sin(syaw);
  double axg = 0.0;  
  double ayg = 0.0;  

  Trajectory traj;
  bool path_found = false;

  for (double T = MIN_T; T <= MAX_T; T += MIN_T) {
      QuinticPolynomial xqp(sx, vxs, axs, gx, vxg, axg, T);
      QuinticPolynomial yqp(sy, vys, ays, gy, vyg, ayg, T);

      traj.time.clear();
      traj.x.clear();
      traj.y.clear();
      traj.yaw.clear();
      traj.v.clear();
      traj.a.clear();
      traj.j.clear();
      traj.curve.clear();
      for (double t = 0.0; t <= T ; t += dt) {
          traj.time.push_back(t);
          traj.x.push_back(xqp.calc_point(t));
          traj.y.push_back(yqp.calc_point(t));
          //test
          //std::cout<<"x"<<xqp.calc_point(t)<<" y"<<yqp.calc_point(t)<<std::endl;

          double vx = xqp.calc_first_derivative(t);
          double vy = yqp.calc_first_derivative(t);
          double v = hypot(vx, vy);
          double yaw = atan2(vy, vx);
          traj.v.push_back(v);
          traj.yaw.push_back(yaw);

          double ax = xqp.calc_second_derivative(t);
          double ay = yqp.calc_second_derivative(t);
          double a = hypot(ax, ay);
          if (traj.v.size() >= 2 && (traj.v.back() - traj.v[traj.v.size() - static_cast<size_t>(2)] < 0.0)) {
              a *= -1;
          }
          traj.a.push_back(a);

          double jx = xqp.calc_third_derivative(t);
          double jy = yqp.calc_third_derivative(t);
          double j = hypot(jx, jy);
          if (traj.a.size() >= 2 && (traj.a.back() - traj.a[traj.a.size() - 2] < 0.0)) {
              j *= -1;
          }
          traj.j.push_back(j);

          double curvature = (vx * ay - vy * ax) / (pow(v, 3) + 0.0001);
          traj.curve.push_back(curvature);
      }

      // Check constraints
      bool accel_ok = true;
      bool jerk_ok = true;

      for (double a : traj.a) {
          if (abs(a) > max_accel) {
              accel_ok = false;
              break;
          }
      }

      for (double j : traj.j) {
          if (abs(j) > max_jerk) {
              jerk_ok = false;
              break;
          }
      }

      if (accel_ok && jerk_ok) {
          path_found = true;
          break;
      }
  }
  if (!path_found) {
      NTERROR << "Failed to find a feasible path, but still try to generate a trajectory with max_time" ;
  }
  
  return traj;
}

QuinticPolynomialPlanner::ResampledPath QuinticPolynomialPlanner::resample_path(const std::vector<double>& rx, const std::vector<double>& ry, const std::vector<double>& rv, const std::vector<double>& ra, const std::vector<double>& ryaw, const std::vector<double>& rcurve) {

  if (rx.size() != ry.size()) {
   
      NTERROR << "rx.size() != ry.size()" ;
  }
  if (spacing <= 0) {
    
      NTERROR << "param spacin <= 0" ;
      spacing = 0.2;
  }
  // Calculate cumulative distances
  std::vector<double> distances = {0.0};
  for (size_t i = 1; i < rx.size(); ++i) {
      double dx = rx[i] - rx[i - 1];
      double dy = ry[i] - ry[i - 1];
      distances.push_back(distances.back() + hypot(dx, dy));
  }
  double total_length = distances.back();
  if (total_length == 0) {
      return {rx, ry, rv, ra, ryaw, rcurve, rx.size()};
  }

  // Generate equally spaced points
  std::vector<double> resampled_rx = {rx[0]};
  std::vector<double> resampled_ry = {ry[0]};
  std::vector<double> resampled_rv = {rv[0]};
  std::vector<double> resampled_ra = {ra[0]};
  std::vector<double> resampled_ryaw = {syaw};
  std::vector<double> resampled_rcurve = {rcurve[0]};

  for (double current_dist = spacing; current_dist < total_length; current_dist += spacing) {
      // Find the segment where current_dist lies
      size_t i = 0;
      while (i < distances.size() - 1 && distances[i + 1] < current_dist) {
          ++i;
      }

      // Linear interpolation
      double segment_length = distances[i + 1] - distances[i];
      if (segment_length > 0) {
          double ratio = (current_dist - distances[i]) / segment_length;
          double x = rx[i] + ratio * (rx[i + 1] - rx[i]);
          double y = ry[i] + ratio * (ry[i + 1] - ry[i]);
          double v = rv[i] + ratio * (rv[i + 1] - rv[i]);
          double v_decreasing = v * (1 - (current_dist / total_length));
          double a = ra[i] + ratio * (ra[i + 1] - ra[i]);
          double curve = rcurve[i] + ratio * (rcurve[i + 1] - rcurve[i]);
          resampled_rx.push_back(x);
          resampled_ry.push_back(y);
          resampled_rv.push_back(v_decreasing);
          resampled_ra.push_back(a);
          resampled_ryaw.push_back(atan2(ry[i + 1] - ry[i], rx[i + 1] - rx[i]));
          resampled_rcurve.push_back(curve);
      }
  }

  // Ensure the goal point is included
  if (hypot(resampled_rx.back() - rx.back(), resampled_ry.back() - ry.back()) > 1e-6) {
      resampled_rx.push_back(rx.back());
      resampled_ry.push_back(ry.back());
      resampled_rv.push_back(0.0);
      resampled_ra.push_back(ra.back());
      resampled_ryaw.push_back(gyaw);
      resampled_rcurve.push_back(rcurve.back());
  }
  // test
  //std::cout<<"resampled_endpoint:"<<resampled_rx.back()<<" "<<resampled_ry.back()<<std::endl;
  return {resampled_rx, resampled_ry, resampled_rv, resampled_ra, resampled_ryaw, resampled_rcurve, resampled_rx.size()};
}

std::vector<double> QuinticPolynomialPlanner::datatransform(const PosePoint& current_pose, const PosePoint& target_pose) {
  // 1. 将target_pose（局部坐标）转换到全局坐标
  // double target_x_global = current_pose.x + target_pose.x * cos(current_pose.yaw) - target_pose.y * sin(current_pose.yaw);
  // double target_y_global = current_pose.y + target_pose.x * sin(current_pose.yaw) + target_pose.y * cos(current_pose.yaw);
  // double target_yaw_global = normalize_angle(current_pose.yaw + target_pose.yaw);

  // test 无需转全局
  double target_x_global = target_pose.x;
  double target_y_global = target_pose.y;
  double target_yaw_global = target_pose.yaw;

  // 2. 将全局坐标的起点和终点转换到车身坐标系（起点为原点）
  double dx = target_x_global - current_pose.x;
  double dy = target_y_global - current_pose.y;
  double gx_local = dx * cos(current_pose.yaw) + dy * sin(current_pose.yaw);
  double gy_local = -dx * sin(current_pose.yaw) + dy * cos(current_pose.yaw);
  double gyaw_local = normalize_angle(target_yaw_global - current_pose.yaw);

  // 返回车身坐标系下的状态
  return {
  0.0, 0.0, 0.0, current_pose.speed, current_pose.acc, gx_local, gy_local, gyaw_local, target_pose.speed, target_pose.acc
  };
}

double QuinticPolynomialPlanner::normalize_angle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

LocalPlannerOutput QuinticPolynomialPlanner::datatransform(const ResampledPath& path) {
  if (path.x.size() > static_cast<size_t>(std::numeric_limits<int64_t>::max())) {
   
    NTERROR << "path.x.size() > static_cast<size_t>(std::numeric_limits<int64_t>::max())" ;
  }

  if (output_tf_frame == 0) {
    LocalTrajectory trajectory;
    bool success = true;
    std::string message = "";
    trajectory.trajectory.clear();
    PosePoint point;
    trajectory.points_cnt = static_cast<int64_t>(path.x.size());
    trajectory.replan_counter = 0; //todo
    for (size_t i = 0; i < path.x.size(); ++i) {
      // 车身坐标 → 全局坐标
      double x_global = current_pose_.x + path.x[i] * cos(current_pose_.yaw) - path.y[i] * sin(current_pose_.yaw);
      double y_global = current_pose_.y + path.x[i] * sin(current_pose_.yaw) + path.y[i] * cos(current_pose_.yaw);
      double yaw_global = normalize_angle(path.yaw[i] + current_pose_.yaw);

      point.x = x_global;
      point.y = y_global;
      point.yaw = yaw_global;
      point.pitch = 0.0;
      point.roll = 0.0;
      point.speed = path.v[i];
      point.curve = path.curve[i]; 
      point.acc = path.a[i];
      point.gear = 3; //todo 默认D档
      trajectory.trajectory.push_back(point);
    }
    NTTRACE<<"path size:"<<trajectory.trajectory.size();
  return {trajectory, success, message};
  }

  if (output_tf_frame == 1) {
    LocalTrajectory trajectory;
    bool success = true;
    std::string message = "";
    trajectory.trajectory.clear();
    PosePoint point;
    trajectory.points_cnt = static_cast<int64_t>(path.x.size());
    trajectory.replan_counter = 0; //todo

    for (size_t i = 0; i < path.x.size(); ++i) {
      point.x = path.x[i];
      point.y = path.y[i];
      point.z = 0.0;
      point.yaw = path.yaw[i];
      point.pitch = 0.0;
      point.roll = 0.0;
      point.speed = path.v[i];
      point.curve = path.curve[i]; 
      point.acc = path.a[i];
      point.gear = 3; //todo 默认D档
      trajectory.trajectory.push_back(point);
    }
    NTTRACE<<"path size:"<<trajectory.trajectory.size();
  return {trajectory, success, message};
  }

  // error
  // NTERROR << "no output_tf_frame, faile to generate output" ;
  LocalTrajectory trajectory;
  memset(&trajectory, 0, sizeof(trajectory));
  return {trajectory, false, "error"};
}


QuinticPolynomialPlanner::QuinticPolynomial::QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double time) {
  // Calculate coefficients of quintic polynomial
  a0 = xs;
  a1 = vxs;
  a2 = axs / 2.0;

  Eigen::Matrix3d A;
  Eigen::Vector3d b;
  Eigen::Vector3d x;

  A << pow(time, 3), pow(time, 4), pow(time, 5),
        3 * pow(time, 2), 4 * pow(time, 3), 5 * pow(time, 4),
        6 * time, 12 * pow(time, 2), 20 * pow(time, 3);

  b << xe - a0 - a1 * time - a2 * pow(time, 2),
        vxe - a1 - 2 * a2 * time,
        axe - 2 * a2;

  x = A.colPivHouseholderQr().solve(b);

  a3 = x(0);
  a4 = x(1);
  a5 = x(2);
}

double QuinticPolynomialPlanner::QuinticPolynomial::calc_point(double t) {
  return a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4) + a5 * pow(t, 5);
}
double QuinticPolynomialPlanner::QuinticPolynomial::calc_first_derivative(double t) {
  return a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) + 5 * a5 * pow(t, 4);
}
double QuinticPolynomialPlanner::QuinticPolynomial::calc_second_derivative(double t) {
  return 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3);
}
double QuinticPolynomialPlanner::QuinticPolynomial::calc_third_derivative(double t) {
  return 6 * a3 + 24 * a4 * t + 60 * a5 * pow(t, 2);
}

TJU_LOCAL_PLANNING_LIB_NAMESPACE_END
