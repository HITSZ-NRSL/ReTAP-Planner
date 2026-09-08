/* Copyright Year: 2026
 * Copyright Owner: Networked Robotics and Systems Lab
 * Authors: Yuxiang Li, Kun Chen, Haoyao Chen
 */

#include "mapping_module/mapping_module.h"

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mapping_module/send_keyframe_data.h"
#include "mapping_module/send_map_data.h"
#include "mapping_module/utils/common_utils.h"
#include "mapping_module/utils/math_utils.h"
#include "mapping_module/utils/octomap_utils.h"
#include "mapping_module/utils/rviz_utils.h"

namespace mapping_module {
MappingModule::MappingModule() {
  ros::NodeHandle nh("/mapping_module");
  ros::NodeHandle nh_local("/mapping_module/local_map");
  ros::NodeHandle nh_global("/mapping_module/global_map");

  std::string local_lidar_topic, global_keyframe_topic, loop_notify_topic;
  nh_local.getParam("octree_enable", local_octree_enable_);
  nh_local.getParam("lidar_topic", local_lidar_topic);
  nh_global.getParam("octree_enable", global_octree_enable_);
  nh_global.getParam("keyframe_topic", global_keyframe_topic);
  nh_global.getParam("terrain_enable", enable_terrain_);
  nh.getParam("odom_topic", odom_topic_);
  nh.getParam("lidar_frame", lidar_frame_);
  nh.getParam("map_frame", map_frame_);
  nh.getParam("transformed_scan", transformed_scan_);
  nh.getParam("loop_notify_topic", loop_notify_topic);
  nh.getParam("loop_enable", enable_loop_);

  world_representation_.reset(new WorldRepresentation());

  Eigen::Isometry3d lidar_tf;
  while (!world_representation_->getSensorPoseEigen(&lidar_tf) && ros::ok()) {
    ROS_WARN(
        "<mapping_module::MappingModule>: Failed to get transform from sensor "
        "tf");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  if (transformed_scan_) {
    trans_odom_pub = nh.advertise<nav_msgs::Odometry>("transformed_odom", 1);
  }

  point_sub_ =
      nh.subscribe("/clicked_point", 1, &MappingModule::clickedPointCallback,
                   this, ros::TransportHints().tcpNoDelay());

  if (local_octree_enable_) {
    lidar_sub_ =
        nh.subscribe(local_lidar_topic, 1, &MappingModule::lidarCallback, this,
                     ros::TransportHints().tcpNoDelay());
  }
  if (global_octree_enable_) {
    key_lidar_sub_ =
        nh.subscribe(global_keyframe_topic, 1, &MappingModule::keyLidarCallback,
                     this, ros::TransportHints().tcpNoDelay());
    //   odom_sub_ = nh.subscribe(odom_topic_, 1, &MappingModule::odomCallback,
    //                                  this,
    //                                  ros::TransportHints().tcpNoDelay());
    if (enable_loop_) {
      query_loop_list_client_ =
          nh.serviceClient<mapping_module::send_keyframe_data>(
              "/send_relo_keyframe_data");
      query_loop_list_client_map_ =
          nh.serviceClient<mapping_module::send_map_data>(
              "/send_relo_map_data");
      loop_closure_sub_ =
          nh.subscribe(loop_notify_topic, 1, &MappingModule::loopCallback, this,
                       ros::TransportHints().tcpNoDelay());
      loop_closure_pub_ = nh.advertise<std_msgs::Int32>(loop_notify_topic, 1);
    }
  }

  if (enable_terrain_) {
    terrain_evaluation_.reset(new TerrainEvaluation(world_representation_));
  }

  if (enable_loop_) {
    loopProcessMaps();
  }
}

void MappingModule::lidarCallback(
    const sensor_msgs::PointCloud2ConstPtr &lidar_msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_pcd(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_pcd_valid(
      new pcl::PointCloud<pcl::PointXYZ>);

  static int cnt = 0;
  if (cnt < 10) {  // 忽略开头10帧
    cnt++;
    return;
  }

  Eigen::Isometry3d current_pose;
  world_representation_->getSensorPoseEigen(&current_pose,
                                            lidar_msg->header.stamp);
  Eigen::Quaterniond q(current_pose.rotation());
  if (!math_utils::isValidFloat(current_pose.translation().x()) ||
      !math_utils::isValidFloat(current_pose.translation().y()) ||
      !math_utils::isValidFloat(current_pose.translation().z()) ||
      !math_utils::isValidFloat(q.x()) || !math_utils::isValidFloat(q.y()) ||
      !math_utils::isValidFloat(q.z()) || !math_utils::isValidFloat(q.w()))
    return;

  pcl::fromROSMsg(*lidar_msg, *local_pcd);
  if (transformed_scan_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_pcd_inv(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*local_pcd,
                             *local_pcd_inv,  // 从全局系变换回传感器坐标系
                             current_pose.inverse().matrix());

    if (0) {  // 旋转雷达点云，考虑雷达倾角
      tf::Quaternion rot =
          tf::createQuaternionFromRPY(0, -30 * M_PI / 180, M_PI);
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.rotate(
          Eigen::Quaterniond(rot.getW(), rot.getX(), rot.getY(), rot.getZ()));
      pcl::transformPointCloud(*local_pcd_inv, *local_pcd,
                               T.matrix());  // 将传感器坐标系下的点云转正
      *local_pcd = *local_pcd_inv;

      // 旋转里程计
      Eigen::Isometry3d current_pose_trans = Eigen::Isometry3d::Identity();
      current_pose_trans.rotate(T.rotation() * current_pose.rotation());
      current_pose_trans.pretranslate(T.rotation() *
                                      current_pose.translation());
      current_pose = current_pose_trans;

      // 发布变换后的odom
      nav_msgs::Odometry msg;
      msg.header.frame_id = map_frame_;
      msg.header.stamp = ros::Time::now();
      msg.pose.pose.position.x = current_pose.translation().x();
      msg.pose.pose.position.y = current_pose.translation().y();
      msg.pose.pose.position.z = current_pose.translation().z();
      Eigen::Quaterniond q(current_pose.rotation());
      msg.pose.pose.orientation.x = q.x();
      msg.pose.pose.orientation.y = q.y();
      msg.pose.pose.orientation.z = q.z();
      msg.pose.pose.orientation.w = q.w();
      trans_odom_pub.publish(msg);
    } else {
      *local_pcd = *local_pcd_inv;
    }
  }
  world_representation_->filterInvalidPoints(local_pcd, local_pcd_valid);
  world_representation_->insertLocalPcdToList(current_pose,
                                              *local_pcd_valid);  // work
}

void MappingModule::keyLidarCallback(
    const sensor_msgs::PointCloud2ConstPtr &lidar_msg) {
  static Eigen::Isometry3d last_pose = Eigen::Isometry3d::Identity();
  static int cnt = 0;
  if (cnt < 10) {  // 忽略开头10帧
    cnt++;
    return;
  }
  Eigen::Isometry3d current_pose;
  world_representation_->getSensorPoseEigen(&current_pose,
                                            lidar_msg->header.stamp);
  Eigen::Quaterniond q(current_pose.rotation());

  if (!math_utils::isValidFloat(current_pose.translation().x()) ||
      !math_utils::isValidFloat(current_pose.translation().y()) ||
      !math_utils::isValidFloat(current_pose.translation().z()) ||
      !math_utils::isValidFloat(q.x()) || !math_utils::isValidFloat(q.y()) ||
      !math_utils::isValidFloat(q.z()) || !math_utils::isValidFloat(q.w()))
    return;

  // if ((current_pose.translation() - last_pose.translation()).norm() < 0.1) {
  //   return;
  // }
  last_pose = current_pose;

  pcl::PointCloud<pcl::PointXYZ>::Ptr local_pcd(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_pcd_valid(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*lidar_msg, *local_pcd);
  if (transformed_scan_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_pcd_inv(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*local_pcd, *local_pcd_inv,
                             current_pose.inverse().matrix());
    if (0) {  // 旋转雷达点云，考虑雷达倾角
      tf::Quaternion rot =
          tf::createQuaternionFromRPY(0, -30 * M_PI / 180, M_PI);
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.rotate(
          Eigen::Quaterniond(rot.getW(), rot.getX(), rot.getY(), rot.getZ()));
      pcl::transformPointCloud(*local_pcd_inv, *local_pcd,
                               T.matrix());  // 将传感器坐标系下的点云转正
      *local_pcd = *local_pcd_inv;

      // 旋转里程计
      Eigen::Isometry3d current_pose_trans = Eigen::Isometry3d::Identity();
      current_pose_trans.rotate(T.rotation() * current_pose.rotation());
      current_pose_trans.pretranslate(T.rotation() *
                                      current_pose.translation());
      current_pose = current_pose_trans;
    } else {
      *local_pcd = *local_pcd_inv;
    }
  }
  world_representation_->filterInvalidPoints(local_pcd, local_pcd_valid);

  world_representation_->insertGlobalPcdToList(
      current_pose, *local_pcd_valid, lidar_msg->header.stamp,
      lidar_msg->header.seq, enable_loop_);  // work

  // std::cout << "<MappingModule::keyLidarCallback>: points="
  //           << local_pcd_valid->size() << std::endl;
}

void MappingModule::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg) {
  static Eigen::Vector3d last_pos(odom_msg->pose.pose.position.x,
                                  odom_msg->pose.pose.position.y,
                                  odom_msg->pose.pose.position.z);
  Eigen::Vector3d curr_pos(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);

  float odomPath_length_ = (last_pos - curr_pos).norm();

  if (odomPath_length_ > 10) {
    last_pos = curr_pos;
    // loopTest();
  }
}

void MappingModule::clickedPointCallback(
    const geometry_msgs::PointStampedConstPtr &point_msg) {
  octomap::point3d selected_point(point_msg->point.x, point_msg->point.y,
                                  point_msg->point.z - 0.1);
  std::lock_guard<std::mutex> lock(world_representation_->globalOctomapMutex());
  octomap::IgTree *octree_ptr_ = world_representation_->getGlobalOctreePtr();
  octomap::OcTreeKey key = octree_ptr_->coordToKey(selected_point);
  octomap::IgTreeNode *node = octree_ptr_->search(key);
  if (node) {
    if (octree_ptr_->isNodeOccupied(node)) {
      bool constrain_flag =
          terrain_evaluation_->checkNodeConstraints(node, true);
      bool neighbor_flag = terrain_evaluation_->checkNeighborCells(key, true);
      std::cout << "<clickedPointCallback>: occ voxel. "
                << "satisfy_constrains = " << constrain_flag
                << ", satisfy_neighbor = " << neighbor_flag
                << ", collision = " << node->getCollision()
                << ", inDrivableSet = "
                << terrain_evaluation_->checkDrivable(key) << std::endl;
      if (constrain_flag && neighbor_flag) {
        std::cout << "set to drivable" << std::endl;
        node->setCollision(false);
      }
    } else {
      std::cout << "<clickedPointCallback>: free voxel" << std::endl;
    }
  } else {
    std::cout << "<clickedPointCallback>: unknown voxel" << std::endl;
  }

  if (0) {
    auto node = octree_ptr_->search(octree_ptr_->coordToKey(selected_point));
    // for Traversibility evaluation
    if (node && octree_ptr_->isNodeOccupied(node)) {
      float slope = node->getSlope();
      float roughness = node->getRoughness();
      float sparsity = node->getSparsity();
      float height_diff = node->getHeightDiff();

      std::string file_path =
          ros::package::getPath("mapping_module") + "/record.txt";
      std::ofstream out(file_path, std::ios::app);  // 追加写入
      std::stringstream ss;
      ss << "point.x\t"
         << "point.y\t"
         << "point.z\t"
         << "slope\t"
         << "roughness\t"
         << "sparsity\t"
         << "height_diff\t" << std::endl;
      ss << selected_point.x() << "\t" << selected_point.y() << "\t"
         << selected_point.z() << "\t" << slope << "\t" << roughness << "\t"
         << sparsity << "\t" << height_diff << "\n";
      out << ss.str();
      ROS_INFO_STREAM("<MappingModule::clickedPointCallback>: " << ss.str());
    } else {
      ROS_ERROR_STREAM(
          "<MappingModule::clickedPointCallback>: " << selected_point);
    }
  }
}

void MappingModule::loopProcessFrames() {
  mapping_module::send_keyframe_data srv;
  srv.request.send_data_command = 1;
  if (!query_loop_list_client_.call(srv)) {  // 请求服务
    ROS_ERROR("<MappingModule::loopProcessFrames>: query_loop_list_client failed");
    return;
  }

  // world_representation_->setCancel();
  // terrain_evaluation_->setCancel();
  // // world_representation_ reset后程序异常退出
  // terrain_evaluation_.reset();
  // world_representation_.reset();
  // world_representation_.reset(new WorldRepresentation());
  // terrain_evaluation_.reset(new TerrainEvaluation(world_representation_));

  std::lock_guard<std::mutex> lock_local_octree(
      world_representation_->localOctomapMutex());
  std::lock_guard<std::mutex> lock_local_list(
      world_representation_->localCloudListMutex());
  std::lock_guard<std::mutex> lock_global_octree(
      world_representation_->globalOctomapMutex());
  std::lock_guard<std::mutex> lock_global_list(
      world_representation_->globalCloudListMutex());
  std::lock_guard<std::mutex> lock(
      world_representation_->globalUpdateVoxelsListMutex());

  // 停止可通行点server
  terrain_evaluation_->resetTerrainRepresentation(world_representation_);

  // 从txt中读取Records
  std::vector<Record> recs;
  ros::WallTime t1 = ros::WallTime::now();
  world_representation_->getRecordManager().setPath(srv.response.data_path);
  world_representation_->getRecordManager().readAllRecordsFromFile(recs);
  ros::WallDuration d = ros::WallTime::now() - t1;
  std::cout << "<MappingModule::loopCallback>: Read timecost " << d.toSec()
            << "s." << std::endl;
  if (recs.size() <= 0) {
    ROS_ERROR("<MappingModule::loopCallback>: No record loaded, return.");
    return;
  }

  Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
  world_representation_->getSensorPoseEigen(&current_pose);
  if (local_octree_enable_) {  // 重新生成local map（用邻域关键帧
    world_representation_->regenerateLocalOctomap(current_pose, recs);
  }

  if (global_octree_enable_) {  // 重新生成global map（用所有关键帧）
    world_representation_->regenerateGlobalOctomap(recs);
    if (enable_terrain_) {
      // 遍历地图点，存入update_all_voxels
      auto tmp_ptr = world_representation_->getGlobalOcTreeSharedPtr();
      octomap::KeySet update_all_voxels;
      for (auto it = tmp_ptr->begin_leafs(); it != tmp_ptr->end_leafs(); ++it) {
        auto key = it.getKey();
        auto node = tmp_ptr->search(key);
        if (node && tmp_ptr->isNodeOccupied(node)) {
          if (update_all_voxels.count(key) == 0)
            update_all_voxels.insert(it.getKey());
        }
      }

      // 清空disjoint_set和kdtree，重新更新地形属性
      terrain_evaluation_->resetTerrainRepresentation(world_representation_);
      world_representation_->globalUpdateVoxelsList().clear();
      terrain_evaluation_->terrainInit(current_pose);
      octomap::point3d update_center(current_pose.translation().x(),
                                     current_pose.translation().y(),
                                     current_pose.translation().z());
      terrain_evaluation_->updateMapTerrainAttribute(update_all_voxels,
                                                     &update_center);
    }
  }
}

void MappingModule::loopProcessMaps() {
  mapping_module::send_map_data srv;
  srv.request.send_data_command = 1;
  if (!query_loop_list_client_map_.call(srv)) {  // 请求服务
    ROS_ERROR("<MappingModule::loopProcessMaps>: query_loop_list_client failed");
    return;
  }

  // srv.response
  std::string local_map_path = srv.response.LocalMap_path;
  std::string global_map_path = srv.response.GlobalMap_path;;
  pcl::PointCloud<pcl::PointXYZ> local_pcd_map, global_pcd_map;

  std::lock_guard<std::mutex> lock_local_octree(
      world_representation_->localOctomapMutex());
  std::lock_guard<std::mutex> lock_local_list(
      world_representation_->localCloudListMutex());
  std::lock_guard<std::mutex> lock_global_octree(
      world_representation_->globalOctomapMutex());
  std::lock_guard<std::mutex> lock_global_list(
      world_representation_->globalCloudListMutex());
  std::lock_guard<std::mutex> lock(
      world_representation_->globalUpdateVoxelsListMutex());

  // 停止可通行点server
  terrain_evaluation_->resetTerrainRepresentation(world_representation_);

  // 从txt中读取Records
  ros::WallTime t1 = ros::WallTime::now();
  std::cout << "<MappingModule::loopCallback>: Load local map ..." << std::endl;
  pcl::io::loadPCDFile(local_map_path, local_pcd_map);
  std::cout << "<MappingModule::loopCallback>: Load global map ..." << std::endl;
  pcl::io::loadPCDFile(global_map_path, global_pcd_map);
  ros::WallDuration d = ros::WallTime::now() - t1;
  std::cout << "<MappingModule::loopCallback>: Load maps timecost " << d.toSec()
            << "s." << std::endl;
  if (local_pcd_map.size() <= 0 || global_pcd_map.size() <= 0) {
    ROS_ERROR("<MappingModule::loopCallback>: No pcd loaded, return.");
    return;
  }

  Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
  world_representation_->getSensorPoseEigen(&current_pose);
  if (local_octree_enable_) {  // 重新生成local map（用邻域关键帧
    world_representation_->regenerateLocalOctomap(current_pose, local_pcd_map);
  }

  if (global_octree_enable_) {  // 重新生成global map（用所有关键帧）
    world_representation_->regenerateGlobalOctomap(current_pose,
                                                   global_pcd_map);
    if (enable_terrain_) {
      // 遍历地图点，存入update_all_voxels
      auto tmp_ptr = world_representation_->getGlobalOcTreeSharedPtr();
      octomap::KeySet update_all_voxels;
      for (auto it = tmp_ptr->begin_leafs(); it != tmp_ptr->end_leafs(); ++it) {
        auto key = it.getKey();
        auto node = tmp_ptr->search(key);
        if (node && tmp_ptr->isNodeOccupied(node)) {
          if (update_all_voxels.count(key) == 0)
            update_all_voxels.insert(it.getKey());
        }
      }

      // 清空disjoint_set和kdtree，重新更新地形属性
      terrain_evaluation_->resetTerrainRepresentation(world_representation_);
      world_representation_->globalUpdateVoxelsList().clear();
      terrain_evaluation_->terrainInit(current_pose);
      octomap::point3d update_center(current_pose.translation().x(),
                                     current_pose.translation().y(),
                                     current_pose.translation().z());
      terrain_evaluation_->updateMapTerrainAttribute(update_all_voxels,
                                                     &update_center);
    }
  }
}

void MappingModule::loopCallback(const std_msgs::Int32 &msg) {
  if (msg.data == 0) {
    // 重定位开始，建图中止一切动作
    return;
  }
  if (msg.data == 1) {
    // 重定位完成，开始重新合并地图
    loopProcessMaps();
    std_msgs::Int32 msg;
    msg.data = 2;
    loop_closure_pub_.publish(msg);
  }
}
}  // namespace mapping_module
