/**
 * @file tracker_method_base.h
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include "trunk_perception/common/types/object.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

using common::Object;

class TrackerMethodBase {
 public:
  TrackerMethodBase() = default;
  virtual ~TrackerMethodBase() = default;

  /**
   * @brief init tracker method
   *
   * @param config yaml node
   * @param object current detection object
   * @return int
   */
  virtual int Init(const YAML::Node& config, const Object& object) = 0;

  /**
   * @brief tracker predict
   *
   * @param dt time interval from previous frame to current frame
   * @param object_tracked current tracking object
   */
  virtual void Predict(const double dt, Object& object_tracked) = 0;

  /**
   * @brief tracker update
   *
   * @param object current detection object
   * @param object_tracked current tracking object
   */
  virtual void Update(const Object& object, Object& object_tracked) = 0;

  /**
   * @brief transform tracker parameters from previous frame to current frame
   *
   * @param tf transform matrix from previous frame to current frame
   */
  virtual void TransformToCurrent(const Eigen::Isometry3f& tf) = 0;
};

class TrackerMethodRegistry {
 public:
  static TrackerMethodRegistry& Get() {
    static TrackerMethodRegistry instance;
    return instance;
  }

  void Register(const std::string& name, std::function<std::shared_ptr<TrackerMethodBase>()> method) {
    factories[name] = std::move(method);
  }

  std::shared_ptr<TrackerMethodBase> Create(const std::string& name) {
    if (factories.find(name) == factories.end()) {
      return nullptr;
    }
    return factories[name]();
  }

 private:
  std::unordered_map<std::string, std::function<std::shared_ptr<TrackerMethodBase>()>> factories;
};

#define REGISTER_TRACKER_METHOD(NAME, TYPE)                                                               \
  namespace {                                                                                             \
  struct Register##TYPE {                                                                                 \
    Register##TYPE() {                                                                                    \
      auto lambda_func = []() -> std::shared_ptr<TrackerMethodBase> { return std::make_shared<TYPE>(); }; \
      TrackerMethodRegistry::Get().Register(NAME, lambda_func);                                           \
    }                                                                                                     \
  };                                                                                                      \
  static Register##TYPE staticRegister##TYPE;                                                             \
  }

TRUNK_PERCEPTION_LIB_NAMESPACE_END