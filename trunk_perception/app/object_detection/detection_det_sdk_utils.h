/**
 * @file detection_det_sdk_utils.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 跟DetectionNetSDK相关的工具函数
 * @version 0.1
 * @date 2025-01-21
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/common/types/object.h"

#include "detection_net_sdk/data.h"
#include "detection_net_sdk/logging.h"

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

class DetectionNetSDKLogger : public net::NetLogger {
  void log(net::NetLogger::Severity severity, const std::string_view msg) noexcept override {
    switch (severity) {
      case net::NetLogger::Severity::kINTERNAL_ERROR:
        TFATAL << "[DetectionNetSDK] " << msg;
        break;
      case net::NetLogger::Severity::kERROR:
        TERROR << "[DetectionNetSDK] " << msg;
        break;
      case net::NetLogger::Severity::kWARNING:
        TWARNING << "[DetectionNetSDK] " << msg;
        break;
      case net::NetLogger::Severity::kINFO:
        TINFO << "[DetectionNetSDK] " << msg;
        break;
      case net::NetLogger::Severity::kVERBOSE:
        TDEBUG << "[DetectionNetSDK] " << msg;
        break;
    }
  }
};

static std::unordered_map<net::ObjectType, ObjectType> DetectionNetSDKObjectTypeDict = {
    {net::ObjectType::UNKNOWN, ObjectType::UNKNOWN},
    {net::ObjectType::VEHICLE, ObjectType::VEHICLE},
    {net::ObjectType::CYCLIST, ObjectType::BICYCLE},
    {net::ObjectType::TRICYCLE, ObjectType::TRICYCLE},
    {net::ObjectType::PEDESTRIAN, ObjectType::PEDESTRIAN},
    {net::ObjectType::BUS, ObjectType::BUS},
    {net::ObjectType::TRUCK, ObjectType::TRUCK},
    {net::ObjectType::TRUCK_HEAD, ObjectType::TRUCK_HEAD},
    {net::ObjectType::TRAILER, ObjectType::TRAILER},
    {net::ObjectType::ALIEN_VEHICLE, ObjectType::ALIEN_VEHICLE},
    {net::ObjectType::CONE, ObjectType::CONE},
    {net::ObjectType::BARREL, ObjectType::BARREL},
    {net::ObjectType::OTHERS, ObjectType::TRAILER},
};
