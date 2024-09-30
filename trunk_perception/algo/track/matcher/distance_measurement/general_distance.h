/**
 * @file general_distance.h
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-10
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/algo/track/tracklet.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

/**
 * @brief compute location distance for given track and object
 *
 * @param track current tracking object
 * @param object current detection object
 * @param time_diff time diff from current tracking object to current detection object
 * @return float distance result
 */
float LocationDistance(const Tracklet& track, const Object& object, const double time_diff);

/**
 * @brief compute direction distance for given track and object
 *
 * @param track current tracking object
 * @param object current detection object
 * @param time_diff time diff from current tracking object to current detection object
 * @return float distance result
 */
float DirectionDistance(const Tracklet& track, const Object& object, const double time_diff);

/**
 * @brief compute bbox size distance for given track and object
 *
 * @param track current tracking object
 * @param object current detection object
 * @param time_diff time diff from current tracking object to current detection object
 * @return float distance result
 */
float BboxSizeDistance(const Tracklet& track, const Object& object, const double time_diff);

/**
 * @brief compute point num distance for given track and object
 *
 * @param track current tracking object
 * @param object current detection object
 * @param time_diff time diff from current tracking object to current detection object
 * @return float distance result
 */
float PointNumDistance(const Tracklet& track, const Object& object, const double time_diff);

/**
 * @brief compute centroid shift distance for given track and object
 *
 * @param track current tracking object
 * @param object current detection object
 * @param time_diff time diff from current tracking object to current detection object
 * @return float distance result
 */
float CentroidShiftDistance(const Tracklet& track, const Object& object, const double time_diff);

/**
 * @brief compute bbox iou distance for given track and object
 *
 * @param track current tracking object
 * @param object current detection object
 * @param time_diff time diff from current tracking object to current detection object
 * @return float distance result
 */
float BboxIouDistance(const Tracklet& track, const Object& object, const double time_diff);

/**
 * @brief compute box tail distance for given track and object
 *
 * @param track current tracking object
 * @param object current detection object
 * @param time_diff time diff from current tracking object to current detection object
 * @return float distance result
 */
float BboxTailDistance(const Tracklet& track, const Object& object, const double time_diff);

TRUNK_PERCEPTION_LIB_NAMESPACE_END