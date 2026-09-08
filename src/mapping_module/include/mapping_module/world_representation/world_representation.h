/*
 * Created on Wed Sep 22 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <octomap/OcTree.h>
#include <octomap/math/Vector3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <algorithm>
#include <condition_variable>
#include <list>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "mapping_module/ikd-Tree/ikd_Tree_impl.h"
#include "mapping_module/query_bbox_pcd.h"
#include "mapping_module/save_octomap.h"
#include "mapping_module/utils/bounding_box.h"
#include "mapping_module/utils/math_utils.h"
#include "mapping_module/utils/octomap_utils.h"
#include "mapping_module/world_representation/ig_tree.h"
#include "mapping_module/world_representation/scan_record.h"

using PointType = pcl::PointXYZ;
using PointVector = KD_TREE<PointType>::PointVector;

namespace mapping_module {

class WorldRepresentation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<WorldRepresentation>;

  WorldRepresentation();
  ~WorldRepresentation();

  octomap::IgTree *getLocalOctreePtr() { return local_octree_ptr_.get(); }
  std::shared_ptr<octomap::IgTree> getLocalOcTreeSharedPtr() {
    return local_octree_ptr_;
  }

  octomap::IgTree *getGlobalOctreePtr() { return global_octree_ptr_.get(); }
  std::shared_ptr<octomap::IgTree> getGlobalOcTreeSharedPtr() {
    return global_octree_ptr_;
  }

  bool getSensorPoseTF(tf::StampedTransform *tf,
                       ros::Time timestamp = ros::Time(0));

  bool getSensorPoseEigen(Eigen::Isometry3d *sensor_pose,
                          ros::Time timestamp = ros::Time(0));
  bool loadParameters();

  void insertLocalPcdToList(const Eigen::Isometry3d &lidar_pose,
                            const pcl::PointCloud<pcl::PointXYZ> &cloud);
  void insertGlobalPcdToList(const Eigen::Isometry3d &lidar_pose,
                             const pcl::PointCloud<pcl::PointXYZ> &cloud,
                             ros::Time timestamp, uint32_t seq,
                             bool enable_loop);
  std::mutex &localOctomapMutex() const { return local_octree_mutex_; }
  std::mutex &globalOctomapMutex() const { return global_octree_mutex_; }
  std::mutex &localCloudListMutex() const { return local_cloud_list_mutex_; }
  std::mutex &globalCloudListMutex() const { return global_cloud_list_mutex_; }
  std::mutex &globalUpdateVoxelsListMutex() const {
    return global_update_voxels_mutex_;
  }

  octomap::point3d globalUpdatePosition() { return global_update_position_; }
  std::list<octomap::point3d> &globalUpdatePositionList() {
    return global_update_position_list_;
  }
  std::list<std::pair<octomap::KeySet, std::vector<octomap::IgTreeNode *>>> &
  globalUpdateVoxelsList() {
    return global_update_voxels_list_;
  }

  std::pair<octomap::KeySet, std::vector<octomap::IgTreeNode *>> &
  globalUpdateVoxels() {
    return global_update_voxels_;
  }

  std::condition_variable &globalUpdateCondition() const {
    return global_update_condition_;
  }

  void resetLocalOctreePtr();
  void resetGlobalOctreePtr();
  bool savePoints(std::string file_name);
  void reloadFrames();
  void updateFramePoses(std::vector<std::string> &file_names,         // NOLINT
                        std::vector<Eigen::Isometry3d> &lidar_poses,  // NOLINT
                        std::vector<ros::Time> &time_stamps);         // NOLINT
  bool saveFrame(const pcl::PointCloud<pcl::PointXYZ> &pcd,
                 std::string file_name);
  void filterInvalidPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr valid_pc);

  bool isValidPoint(pcl::PointXYZ p) {
    if (!math_utils::isValidFloat(p.x) || !math_utils::isValidFloat(p.y) ||
        !math_utils::isValidFloat(p.z))
      return false;
    return true;
  }
  void regenerateGlobalOctomap(std::vector<Record> &new_records);  // NOLINT
  void regenerateGlobalOctomap(
      Eigen::Isometry3d &curr_pose,                               // NOLINT
      pcl::PointCloud<pcl::PointXYZ> &global_map);                // NOLINT
  void regenerateLocalOctomap(Eigen::Isometry3d &curr_pose,       // NOLINT
                              std::vector<Record> &new_records);  // NOLINT
  void regenerateLocalOctomap(
      Eigen::Isometry3d &curr_pose,                // NOLINT
      pcl::PointCloud<pcl::PointXYZ> &local_map);  // NOLINT
  octomap::Boundingbox getInputPointcloudBbox() { return input_pc_bbox_; }
  RecordManager &getRecordManager() { return record_manager_; }
  void setCancel() {
    std::lock_guard<std::mutex> lock(cancel_mutex_);
    cancel_ = true;
  }

 private:
  ros::Publisher local_octomap_publisher_, global_octomap_publisher_,
      global_std_octomap_publisher_, local_std_octomap_publisher_,
      global_pcd_publisher_, global_update_pcd_publisher_,
      global_slope_pcd_publisher_, global_roughness_pcd_publisher_,
      global_sparsity_pcd_publisher_, global_travasability_pcd_publisher_,
      global_collision_pcd_publisher_, global_incollision_pcd_publisher_,
      global_connected_pcd_publisher_;
  ros::Publisher local_bbox_publisher_, local_pcd_publisher_,
      input_bbox_publisher_, local_bbox_param_publisher_,
      input_bbox_param_publisher_;

  ros::ServiceServer save_octomap_server_;
  ros::ServiceServer query_bbox_pcd_server_;
  std::shared_ptr<octomap::IgTree> local_octree_ptr_, global_octree_ptr_;
  std::shared_ptr<octomap::OcTree> global_octree_std_ptr_,
      local_octree_std_ptr_;
  std::list<std::pair<octomap::Pointcloud, octomap::point3d>> local_cloud_list_,
      global_cloud_list_;
  std::thread local_octomap_pub_thread_, global_octomap_pub_thread_;
  std::thread insert_local_cloud_thread_, insert_global_cloud_thread_;
  std::thread roi_pub_thread_;

  mutable std::mutex local_octree_mutex_, global_octree_mutex_;
  mutable std::mutex local_cloud_list_mutex_, global_cloud_list_mutex_,
      global_update_voxels_mutex_;
  mutable std::condition_variable global_update_condition_;
  std::mutex tf_mutex_;

  std::string map_frame_, lidar_frame_;
  std::string data_folder_;
  bool local_octree_enable_ = false, global_octree_enable_ = false;
  bool global_octree_std_enable_ = false, local_octree_std_enable_ = false,
       enable_loop_ = false, global_pcd_enable_ = false;
  bool local_tsdf_enable_ = false, global_tsdf_enable_ = false,
       enable_terrain_ = false;
  double local_publish_frequency_, global_publish_frequency_;
  double local_octree_resolution_, global_octree_resolution_;

  double local_octree_hit_probability_, local_octree_miss_probability_;
  double global_octree_hit_probability_, global_octree_miss_probability_;

  int local_update_free_, global_update_free_;

  std::pair<octomap::KeySet, std::vector<octomap::IgTreeNode *>>
      global_update_voxels_;
  std::list<std::pair<octomap::KeySet, std::vector<octomap::IgTreeNode *>>>
      global_update_voxels_list_;
  octomap::point3d global_update_position_;
  std::list<octomap::point3d> global_update_position_list_;

  float local_map_size_;
  float local_map_height_;
  float lidar_range_max_ = 10;
  float lidar_range_min_ = 0.5;
  int max_buffer_size_;
  float lidar_tf_dt_ = 0;
  octomap::Boundingbox local_bbox_, input_pc_bbox_;
  KD_TREE<PointType>::Ptr local_kdtree_ptr_;
  bool local_kdtree_initialized_ = false;
  RecordManager record_manager_;

  bool cancel_ = false;
  std::mutex kdtree_mutex_, cancel_mutex_;
  bool checkCancel() {
    std::lock_guard<std::mutex> lock(cancel_mutex_);
    return cancel_;
  }

  void publishGlobalOcTreeThread();
  void publishLocalOcTreeThread();

  void publishROIThread();
  void publishGlobalUpdateBBox();
  void publishLocalUpdateBBox();

  void publishUpdateBBoxThread();

  void insertLocalCloudThread();

  void insertGlobalCloudThread();

  bool saveOctomapServerCallback(save_octomap::Request &request,     // NOLINT
                                 save_octomap::Response &response);  // NOLINT
  bool queryBboxPCDServerCallback(
      query_bbox_pcd::Request &request,     // NOLINT
      query_bbox_pcd::Response &response);  // NOLINT
};
}  // namespace mapping_module
