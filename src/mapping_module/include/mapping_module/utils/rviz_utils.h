/*
 * Created on Wed Dec 16 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#pragma once

#include <geometry_msgs/PoseArray.h>
#include <octomap/octomap_types.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <vector>
#include "mapping_module/utils/bounding_box.h"
namespace rviz_utils {

visualization_msgs::MarkerArray
displayScoresInPoses(const std::vector<geometry_msgs::Pose>& positions,
                         const std::vector<double>& scores);

visualization_msgs::Marker drawBoundingbox(const octomap::Boundingbox &bbox);
visualization_msgs::Marker convertBoundingboxMsg(const octomap::Boundingbox& bbox);
};  // namespace rviz_utils

