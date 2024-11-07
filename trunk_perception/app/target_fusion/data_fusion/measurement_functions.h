/**
 * @file measurement_functions.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 预定义的不同传感器的观测方程
 * @version 0.1
 * @date 2024-10-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <Eigen/Dense>

#include "trunk_perception/common/macros.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace Setting1 {
// 2维1阶
// 状态量 [x, y, vx, vy]
// 后续可以用不同的ns扩展其他的预定义状态空间下的观测方程

// 激光观测: [x, y, vx, vy]
static Eigen::MatrixXd H_lidar = Eigen::MatrixXd::Identity(4, 4);
// clang-format off
static Eigen::MatrixXd R_lidar = (Eigen::MatrixXd(4, 4) << 
    0.1, 0,   0,   0,
    0,   0.1, 0,   0,
    0,   0,   0.2, 0,
    0,   0,   0,   0.2).finished();
// clang-format on

// 毫米波雷达观测: [x, y, vx, vy]
static Eigen::MatrixXd H_radar0 = Eigen::MatrixXd::Identity(4, 4);
// clang-format off
static Eigen::MatrixXd R_radar0 = (Eigen::MatrixXd(4, 4) << 
    10.0, 0,   0,   0,
    0,   10.0, 0,   0,
    0,   0,   0.1, 0,
    0,   0,   0,   0.1).finished();
// clang-format on

static std::unordered_map<std::string, Eigen::MatrixXd> R = {
    {"Lidar", R_lidar},
    {"Radar0", R_radar0},
};

static std::unordered_map<std::string, Eigen::MatrixXd> H = {
    {"Lidar", H_lidar},
    {"Radar0", H_radar0},
};

}  // namespace Setting1

static std::unordered_map<std::string, Eigen::MatrixXd> GetMeasurementR(const std::string& setting) {
  if (setting == "Setting1") {
    return Setting1::R;
  }
  TERROR << "MeasurementR setting not found: " << setting;
  return Setting1::R;
}

static std::unordered_map<std::string, Eigen::MatrixXd> GetMeasurementH(const std::string& setting) {
  if (setting == "Setting1") {
    return Setting1::H;
  }
  TERROR << "MeasurementH setting not found: " << setting;
  return Setting1::H;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END