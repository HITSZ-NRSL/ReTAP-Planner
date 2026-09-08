/*
 * Created on Wed Sep 22 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */
#pragma once

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>


#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "mapping_module/terrain_evaluation.h"
#include "mapping_module/utils/bounding_box.h"
#include "mapping_module/world_representation/world_representation.h"

namespace mapping_module {
class MappingModule {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MappingModule();

 private:
  std::shared_ptr<WorldRepresentation> world_representation_;
  std::shared_ptr<TerrainEvaluation> terrain_evaluation_;

 private:
  void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &lidar_msg);
  void keyLidarCallback(const sensor_msgs::PointCloud2ConstPtr &lidar_msg);
  void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
  void loopCallback(const std_msgs::Int32 &msg);
  void loopProcessFrames();
  void loopProcessMaps();

  void clickedPointCallback(
      const geometry_msgs::PointStampedConstPtr &point_msg);

  ros::Subscriber lidar_sub_, key_lidar_sub_, odom_sub_, loop_closure_sub_,
      point_sub_;
  std::string lidar_frame_, map_frame_, base_frame_, odom_topic_, lidar_topic_;
  float lidar_tf_dt_ = 0;
  bool enable_loop_ = false;
  bool enable_terrain_ = false;
  bool transformed_scan_ = false;
  bool local_octree_enable_ = false;
  bool global_octree_enable_ = false;

  ros::Publisher trans_odom_pub, loop_closure_pub_;
  ros::ServiceClient query_loop_list_client_, query_loop_list_client_map_;
};
};  // namespace mapping_module
