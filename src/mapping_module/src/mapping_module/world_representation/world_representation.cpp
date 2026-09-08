/* Copyright Year: 2026
 * Copyright Owner: Networked Robotics and Systems Lab
 * Authors: Yuxiang Li, Kun Chen, Haoyao Chen
 */

#include "mapping_module/world_representation/world_representation.h"

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>

#include <algorithm>
#include <fstream>

#include "mapping_module/utils/common_utils.h"
#include "mapping_module/utils/file_path.h"
#include "mapping_module/utils/rviz_utils.h"

namespace mapping_module {
WorldRepresentation::WorldRepresentation() {
  loadParameters();

  ros::NodeHandle nh("mapping_module");

  if (local_octree_enable_) {
    local_octomap_publisher_ =
        nh.advertise<octomap_msgs::Octomap>("local_octree", 1);
    local_std_octomap_publisher_ =
        nh.advertise<octomap_msgs::Octomap>("local_std_octree", 1);
    insert_local_cloud_thread_ =
        std::thread(&WorldRepresentation::insertLocalCloudThread, this);
    local_octomap_pub_thread_ =
        std::thread(&WorldRepresentation::publishLocalOcTreeThread, this);
    local_bbox_publisher_ =
        nh.advertise<visualization_msgs::Marker>("local_update_bbox", 1);
    local_bbox_param_publisher_ =
        nh.advertise<visualization_msgs::Marker>("local_update_bbox_param", 1);
    local_pcd_publisher_ =
        nh.advertise<sensor_msgs::PointCloud2>("local_pointcloud", 1);
    local_kdtree_ptr_.reset(new KD_TREE<pcl::PointXYZ>(0.3, 0.6, 0.025));
  }

  if (global_octree_enable_) {
    global_octomap_publisher_ =
        nh.advertise<octomap_msgs::Octomap>("global_octree", 1);
    global_std_octomap_publisher_ =
        nh.advertise<octomap_msgs::Octomap>("global_std_octree", 1);
    global_slope_pcd_publisher_ =
        nh.advertise<sensor_msgs::PointCloud2>("global_slope_pointcloud", 1);
    global_roughness_pcd_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(
        "global_roughness_pointcloud", 1);
    global_sparsity_pcd_publisher_ =
        nh.advertise<sensor_msgs::PointCloud2>("global_sparsity_pointcloud", 1);
    global_travasability_pcd_publisher_ =
        nh.advertise<sensor_msgs::PointCloud2>(
            "global_travasability_pointcloud", 1);
    global_collision_pcd_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(
        "global_collision_pointcloud", 1);
    global_incollision_pcd_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(
        "global_incollision_pointcloud", 1);
    global_connected_pcd_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(
        "global_connected_pointcloud", 1);
    // global_update_pcd_publisher_ =
    //     nh.advertise<sensor_msgs::PointCloud2>("global_pointcloud", 1);
    insert_global_cloud_thread_ =
        std::thread(&WorldRepresentation::insertGlobalCloudThread, this);
    global_octomap_pub_thread_ =
        std::thread(&WorldRepresentation::publishGlobalOcTreeThread, this);
    input_bbox_publisher_ =
        nh.advertise<visualization_msgs::Marker>("global_update_bbox", 1);
    input_bbox_param_publisher_ =
        nh.advertise<visualization_msgs::Marker>("global_update_bbox_param", 1);
  }

  save_octomap_server_ = nh.advertiseService(
      "save_octomap", &WorldRepresentation::saveOctomapServerCallback, this);
  query_bbox_pcd_server_ = nh.advertiseService(
      "query_bbox_pcd", &WorldRepresentation::queryBboxPCDServerCallback, this);
  std::cout << "<WorldRepresentation>: started" << std::endl;
}

WorldRepresentation::~WorldRepresentation() {
  setCancel();
  // save_octomap_server_.shutdown();
  // query_bbox_pcd_server_.shutdown();
  if (local_octree_enable_) {
    local_octomap_pub_thread_.join();
    insert_local_cloud_thread_.join();
  }
  if (global_octree_enable_) {
    global_octomap_pub_thread_.join();
    insert_global_cloud_thread_.join();
  }

  printf("<~WorldRepresentation>: All threads exit successfully.\n");
  if (local_octree_ptr_) local_octree_ptr_->clear();
  if (local_octree_std_ptr_) local_octree_std_ptr_->clear();
  if (global_octree_ptr_) global_octree_ptr_->clear();
  if (global_octree_std_ptr_) global_octree_std_ptr_->clear();
  printf("<~WorldRepresentation>: All octrees have been cleared.\b");
}

void WorldRepresentation::resetLocalOctreePtr() {
  if (local_octree_ptr_) local_octree_ptr_->clear();
  local_octree_ptr_.reset(new octomap::IgTree(local_octree_resolution_));
  local_octree_ptr_->enableChangeDetection(true);
  local_octree_ptr_->setProbHit(local_octree_hit_probability_);
  local_octree_ptr_->setProbMiss(local_octree_miss_probability_);

  if (local_octree_std_enable_) {
    if (local_octree_std_ptr_) local_octree_std_ptr_->clear();
    local_octree_std_ptr_.reset(new octomap::OcTree(local_octree_resolution_));
  }
}

void WorldRepresentation::resetGlobalOctreePtr() {
  if (global_octree_ptr_) global_octree_ptr_->clear();
  global_octree_ptr_.reset(new octomap::IgTree(global_octree_resolution_));
  global_octree_ptr_->enableChangeDetection(true);
  global_octree_ptr_->setProbHit(global_octree_hit_probability_);
  global_octree_ptr_->setProbMiss(global_octree_miss_probability_);

  if (global_octree_std_enable_) {
    if (global_octree_std_ptr_) global_octree_std_ptr_->clear();
    global_octree_std_ptr_.reset(
        new octomap::OcTree(global_octree_resolution_));
  }
}

bool WorldRepresentation::loadParameters() {
  ros::NodeHandle nh("/mapping_module");
  nh.getParam("lidar_range_max", lidar_range_max_);
  nh.getParam("lidar_range_min", lidar_range_min_);
  nh.getParam("map_frame", map_frame_);
  nh.getParam("lidar_frame", lidar_frame_);
  nh.getParam("max_buffer_size", max_buffer_size_);
  nh.getParam("lidar_tf_dt", lidar_tf_dt_);

  ros::NodeHandle nh_local("/mapping_module/local_map");
  nh_local.getParam("octree_enable", local_octree_enable_);
  nh_local.param("octree_hit_probability", local_octree_hit_probability_, 0.7);
  nh_local.param("octree_miss_probability", local_octree_miss_probability_,
                 0.4);
  nh_local.getParam("octree_resolution", local_octree_resolution_);
  nh_local.getParam("publish_frequency", local_publish_frequency_);
  nh_local.getParam("std_octree_enable", local_octree_std_enable_);
  nh_local.getParam("map_size", local_map_size_);
  nh_local.getParam("map_height", local_map_height_);
  nh_local.getParam("update_free", local_update_free_);
  nh_local.getParam("tsdf_enable", local_tsdf_enable_);

  ros::NodeHandle nh_global("/mapping_module/global_map");
  nh_global.getParam("octree_enable", global_octree_enable_);
  nh_global.getParam("octree_resolution", global_octree_resolution_);
  nh_global.param("octree_hit_probability", global_octree_hit_probability_,
                  0.7);
  nh_global.param("octree_miss_probability", global_octree_miss_probability_,
                  0.4);
  nh_global.getParam("octree_resolution", global_octree_resolution_);
  nh_global.getParam("publish_frequency", global_publish_frequency_);
  nh_global.getParam("std_octree_enable", global_octree_std_enable_);
  nh_global.getParam("loop_enable", enable_loop_);
  nh_global.getParam("update_free", global_update_free_);
  nh_global.getParam("tsdf_enable", global_tsdf_enable_);
  nh_global.getParam("terrain_enable", enable_terrain_);
  nh_global.getParam("pcd_enable", global_pcd_enable_);

  if (local_octree_enable_) resetLocalOctreePtr();
  if (global_octree_enable_) resetGlobalOctreePtr();

  if (enable_loop_) {
    data_folder_ = ros::package::getPath("mapping_module") +
                   std::string("/../../data/") +
                   stampToDateString(ros::WallTime::now());
    std::string cmd = "mkdir -p \"" + data_folder_ + "\"";
    int ret = system(cmd.c_str());

    record_manager_.setPath(data_folder_ + "\record.txt");
  }

  return true;
}

bool WorldRepresentation::getSensorPoseTF(tf::StampedTransform *sensor_pose,
                                          ros::Time timestamp) {
  std::lock_guard<std::mutex> lock_local(tf_mutex_);
  static tf::TransformListener listener;
  try {
    listener.waitForTransform(map_frame_, lidar_frame_, timestamp,
                              ros::Duration(lidar_tf_dt_));
    listener.lookupTransform(map_frame_, lidar_frame_, timestamp, *sensor_pose);
  } catch (...) {
    ROS_ERROR("Listen TF [%.3f] (%s -> %s) timeout!", timestamp.toSec(),
              map_frame_.c_str(), lidar_frame_.c_str());
    return false;
  }
  return true;
}

bool WorldRepresentation::getSensorPoseEigen(Eigen::Isometry3d *sensor_pose,
                                             ros::Time timestamp) {
  tf::StampedTransform lidar_transform_tf;
  if (!getSensorPoseTF(&lidar_transform_tf, timestamp)) {
    return false;
  }
  tf::transformTFToEigen(lidar_transform_tf, *sensor_pose);

  return true;
}

void WorldRepresentation::filterInvalidPoints(
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
    pcl::PointCloud<pcl::PointXYZ>::Ptr valid_pc) {
  valid_pc->clear();
  for (auto p : pc->points) {
    if (!isValidPoint(p)) continue;

    float dis = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (dis < lidar_range_min_) continue;
    // if (pt.norm() > lidar_range_max_) continue;

    valid_pc->push_back(p);
  }
}

// 存入local_cloud_list_，等待insertCloudThread线程插入到local_octree_ptr_
void WorldRepresentation::insertLocalPcdToList(
    const Eigen::Isometry3d &lidar_pose,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  std::lock_guard<std::mutex> lock_local(local_cloud_list_mutex_);
  //   if (local_cloud_list_.size() > max_buffer_size_) return;
  octomap::point3d lidar_origin(lidar_pose.translation().x(),
                                lidar_pose.translation().y(),
                                lidar_pose.translation().z());
  pcl::PointCloud<pcl::PointXYZ> world_pcd;
  pcl::transformPointCloud(cloud, world_pcd, lidar_pose.matrix());

  octomap::Pointcloud cloud_octomap;
  octomap::pclToOctomap(world_pcd, &cloud_octomap);
  // std::cout << "<insertLocalPcdToList>: append "
  //           << world_pcd.size() << " points\n";

  local_cloud_list_.emplace_back(cloud_octomap, lidar_origin);
}

void WorldRepresentation::insertGlobalPcdToList(
    const Eigen::Isometry3d &lidar_pose,
    const pcl::PointCloud<pcl::PointXYZ> &cloud, ros::Time timestamp,
    uint32_t seq, bool enable_loop) {
  std::lock_guard<std::mutex> lock_global(global_cloud_list_mutex_);

  // if (enable_loop) {
  //   // 保存frame到文件、并追加txt记录
  //   std::string frame_path = utils::PathJoin(
  //       data_folder_, stampToDateString(timestamp) + "_bin.pcd");
  //   saveFrame(cloud, frame_path);
  //   Record rec(seq, timestamp, lidar_pose, frame_path);
  //   record_manager_.writeOneRecordToFile(rec);
  // }

  octomap::point3d lidar_origin(lidar_pose.translation().x(),
                                lidar_pose.translation().y(),
                                lidar_pose.translation().z());
  pcl::PointCloud<pcl::PointXYZ> world_pcd;
  pcl::transformPointCloud(cloud, world_pcd, lidar_pose.matrix());

  octomap::Pointcloud cloud_octomap;
  octomap::pclToOctomap(world_pcd, &cloud_octomap);

  global_cloud_list_.emplace_back(cloud_octomap, lidar_origin);
}

void WorldRepresentation::publishLocalOcTreeThread() {
  ros::Rate rate(local_publish_frequency_);
  sleep(2);
  while (ros::ok() && !checkCancel()) {
    octomap_msgs::Octomap octree_msg;
    std::shared_ptr<octomap::IgTree> tmp_ptr;
    bool publish_local_enable =
        (local_octree_enable_ &&
         local_octomap_publisher_.getNumSubscribers() > 0);
    bool publish_local_std_enable =
        (local_octree_std_enable_ &&
         local_std_octomap_publisher_.getNumSubscribers() > 0);
    if (publish_local_enable || publish_local_std_enable) {
      std::lock_guard<std::mutex> lock(local_octree_mutex_);
      if (local_octree_ptr_) {
        tmp_ptr = local_octree_ptr_->deepClone();
        octree_msg.header.stamp = ros::Time::now();
        octree_msg.header.frame_id = map_frame_;
      }
    }

    if (tmp_ptr) {
      if (publish_local_enable) {
        octomap_msgs::fullMapToMsg(*tmp_ptr, octree_msg);
        local_octomap_publisher_.publish(octree_msg);
      }
      if (publish_local_std_enable) {
        local_octree_std_ptr_.reset(
            new octomap::OcTree(local_octree_resolution_));
        for (auto it = tmp_ptr->begin_leafs(); it != tmp_ptr->end_leafs();
             ++it) {
          octomap::OcTreeKey key = it.getKey();
          octomap::IgTreeNode *node = tmp_ptr->search(key);
          if (!node) continue;
          local_octree_std_ptr_->updateNode(key, tmp_ptr->isNodeOccupied(node));
        }

        octomap_msgs::fullMapToMsg(*local_octree_std_ptr_, octree_msg);
        local_std_octomap_publisher_.publish(octree_msg);
        local_octree_std_ptr_->clear();
      }
      tmp_ptr->clear();
    }
    rate.sleep();
  }
  std::cout << "<publishLocalOcTreeThread>: thread exits." << std::endl;
}

void WorldRepresentation::publishGlobalOcTreeThread() {
  sleep(2);
  ros::Rate rate(global_publish_frequency_);
  std::once_flag flag;
  while (ros::ok() && !checkCancel()) {
    octomap_msgs::Octomap octree_msg;
    std::shared_ptr<octomap::IgTree> tmp_ptr;
    bool publish_global_enable =
        (global_octree_enable_ &&
         global_octomap_publisher_.getNumSubscribers() > 0);
    bool publish_global_std_enable =
        (global_octree_std_enable_ &&
         global_std_octomap_publisher_.getNumSubscribers() > 0);
    if (publish_global_enable || publish_global_std_enable ||
        global_pcd_enable_) {
      std::lock_guard<std::mutex> lock(global_octree_mutex_);
      if (global_octree_ptr_) {
        tmp_ptr = global_octree_ptr_->deepClone();
        octree_msg.header.stamp = ros::Time::now();
        octree_msg.header.frame_id = map_frame_;
      }
    }
    if (tmp_ptr) {
      if (publish_global_enable) {
        octomap_msgs::fullMapToMsg(*tmp_ptr, octree_msg);
        global_octomap_publisher_.publish(octree_msg);
      }
      if (publish_global_std_enable) {
        global_octree_std_ptr_.reset(
            new octomap::OcTree(global_octree_resolution_));
        for (auto it = tmp_ptr->begin_leafs(); it != tmp_ptr->end_leafs();
             ++it) {
          octomap::OcTreeKey key = it.getKey();
          octomap::IgTreeNode *node = tmp_ptr->search(key);
          if (!node) continue;
          global_octree_std_ptr_->updateNode(key,
                                             tmp_ptr->isNodeOccupied(node));
        }

        octomap_msgs::fullMapToMsg(*global_octree_std_ptr_, octree_msg);
        global_std_octomap_publisher_.publish(octree_msg);
        global_octree_std_ptr_->clear();
      }
      if (global_pcd_enable_) {
        pcl::PointCloud<pcl::PointXYZI> slope_pcd, roughness_pcd, sparsity_pcd,
            traversability_pcd, collision_pcd, incollision_pcd, connected_pcd;
        for (auto it = tmp_ptr->begin_leafs(); it != tmp_ptr->end_leafs();
             ++it) {
          octomap::OcTreeKey key = it.getKey();
          octomap::IgTreeNode *node = tmp_ptr->search(key);
          if (!node || !tmp_ptr->isNodeOccupied(node)) continue;
          auto p3d = tmp_ptr->keyToCoord(key);
          pcl::PointXYZI p;
          p.x = p3d.x();
          p.y = p3d.y();
          p.z = p3d.z();
          p.intensity = node->getSlope();
          slope_pcd.push_back(p);
          p.intensity = node->getRoughness();
          roughness_pcd.push_back(p);
          p.intensity = node->getSparsity();
          sparsity_pcd.push_back(p);
          p.intensity = node->getTraversability();
          traversability_pcd.push_back(p);
          if (node->getCollision()) {
            p.intensity = 1;
            collision_pcd.push_back(p);
          } else {
            p.intensity = 0.5;
            incollision_pcd.push_back(p);
          }
          p.intensity = node->getConnected() ? 0.5 : 1;
          connected_pcd.push_back(p);
        }
        if (slope_pcd.size() > 0) {
          sensor_msgs::PointCloud2 msg;
          pcl::toROSMsg(slope_pcd, msg);
          msg.header.frame_id = map_frame_;
          msg.header.stamp = ros::Time::now();
          global_slope_pcd_publisher_.publish(msg);

          pcl::toROSMsg(roughness_pcd, msg);
          msg.header.frame_id = map_frame_;
          msg.header.stamp = ros::Time::now();
          global_roughness_pcd_publisher_.publish(msg);

          pcl::toROSMsg(sparsity_pcd, msg);
          msg.header.frame_id = map_frame_;
          msg.header.stamp = ros::Time::now();
          global_sparsity_pcd_publisher_.publish(msg);

          pcl::toROSMsg(traversability_pcd, msg);
          msg.header.frame_id = map_frame_;
          msg.header.stamp = ros::Time::now();
          global_travasability_pcd_publisher_.publish(msg);

          pcl::toROSMsg(collision_pcd, msg);
          msg.header.frame_id = map_frame_;
          msg.header.stamp = ros::Time::now();
          global_collision_pcd_publisher_.publish(msg);

          pcl::toROSMsg(incollision_pcd, msg);
          msg.header.frame_id = map_frame_;
          msg.header.stamp = ros::Time::now();
          global_incollision_pcd_publisher_.publish(msg);

          pcl::toROSMsg(connected_pcd, msg);
          msg.header.frame_id = map_frame_;
          msg.header.stamp = ros::Time::now();
          global_connected_pcd_publisher_.publish(msg);
        }
      }
      tmp_ptr->clear();
    }

    rate.sleep();
  }
  std::cout << "<publishGlobalOcTreeThread>: thread exits." << std::endl;
}

void WorldRepresentation::insertLocalCloudThread() {
  ros::Rate rate(100);
  while (ros::ok() && !checkCancel()) {
    int list_size = 0;
    {  // lock 作用域
      std::lock_guard<std::mutex> lock(local_cloud_list_mutex_);
      list_size = local_cloud_list_.size();
    }
    if (list_size != 0) {
      std::lock_guard<std::mutex> lock_octree(local_octree_mutex_);
      std::lock_guard<std::mutex> lock(local_cloud_list_mutex_);

      if (local_cloud_list_.size() == 0)  // 2nd check, avoid &p to be null
        continue;
      while (local_cloud_list_.size() > max_buffer_size_)
        local_cloud_list_.pop_front();

      const std::pair<octomap::Pointcloud, octomap::point3d> &p =
          local_cloud_list_.front();

      ros::WallTime t1 = ros::WallTime::now();
      octomap::Boundingbox local_bbox;
      local_bbox.insertPoint(
          octomap::point3d(local_map_size_ / 2.0 + p.second.x(),
                           local_map_size_ / 2.0 + p.second.y(),
                           local_map_height_ / 2.0 + p.second.z()));
      local_bbox.insertPoint(
          octomap::point3d(-local_map_size_ / 2.0 + p.second.x(),
                           -local_map_size_ / 2.0 + p.second.y(),
                           -local_map_height_ / 2.0 + p.second.z()));
      local_bbox_ = local_bbox;
      // 局部更新时，给定当前位置的滑窗
      if (1) {
        local_octree_ptr_->deleteNodesOutsideBbox(local_bbox, 1.3);
        local_octree_ptr_->insertPointCloud(p.first, p.second, local_bbox,
                                            local_update_free_,
                                            lidar_range_max_);
        ros::WallDuration d = ros::WallTime::now() - t1;
        std::cout << "<insertLocalCloudThread>: bbox=" << local_bbox
                  << "pts: " << p.first.size() << ". timecost: " << d.toSec()
                  << "s" << std::endl;
      } else {  // kd-tree update
        t1 = ros::WallTime::now();
        // 先插入kd-tree
        PointVector pcd_vector;
        for (auto it : p.first) {
          pcd_vector.push_back(pcl::PointXYZ(it.x(), it.y(), it.z()));
        }
        if (!local_kdtree_initialized_ && pcd_vector.size() > 0) {
          local_kdtree_initialized_ = true;
          local_kdtree_ptr_->Build(pcd_vector);
        } else {
          local_kdtree_ptr_->Add_Points(pcd_vector, true);
          // 再删掉框外的
          float delta = 0.5;
          std::vector<BoxPointType> outer_bboxes;
          BoxPointType upper_bbox = {
              {local_bbox.minX() - delta, local_bbox.minY() - delta,
               local_bbox.maxZ()},
              {local_bbox.maxX() + delta, local_bbox.maxY() + delta,
               local_bbox.maxZ() + delta}};
          outer_bboxes.push_back(upper_bbox);
          BoxPointType lower_bbox = {
              {local_bbox.minX() - delta, local_bbox.minY() - delta,
               local_bbox.minZ() - delta},
              {local_bbox.maxX() + delta, local_bbox.maxY() + delta,
               local_bbox.minZ()}};
          outer_bboxes.push_back(lower_bbox);
          BoxPointType rear_bbox = {
              {local_bbox.minX() - delta, local_bbox.minY() - delta,
               local_bbox.minZ()},
              {local_bbox.minX(), local_bbox.maxY() + delta,
               local_bbox.maxZ()}};
          outer_bboxes.push_back(rear_bbox);
          BoxPointType front_bbox = {
              {local_bbox.maxX(), local_bbox.minY() - delta, local_bbox.minZ()},
              {local_bbox.maxX() + delta, local_bbox.maxY() + delta,
               local_bbox.maxZ()}};
          outer_bboxes.push_back(front_bbox);
          BoxPointType left_bbox = {
              {local_bbox.minX(), local_bbox.minY() - delta, local_bbox.minZ()},
              {local_bbox.maxX(), local_bbox.minY(), local_bbox.maxZ()}};
          outer_bboxes.push_back(left_bbox);
          BoxPointType right_bbox = {
              {local_bbox.minX(), local_bbox.maxY(), local_bbox.minZ()},
              {local_bbox.maxX(), local_bbox.maxY() + delta,
               local_bbox.maxZ()}};
          outer_bboxes.push_back(right_bbox);
          local_kdtree_ptr_->Delete_Point_Boxes(outer_bboxes);
          ros::WallDuration d = ros::WallTime::now() - t1;
          std::cout << "<insertLocalCloudThread>: bbox=" << local_bbox
                    << "pts: " << p.first.size() << ". timecost: " << d.toSec()
                    << "s, kdtree" << std::endl;
          // 查询并发布
          t1 = ros::WallTime::now();
          PointVector local_pts;
          BoxPointType search_box = {
              {local_bbox.minX(), local_bbox.minY(), local_bbox.minZ()},
              {local_bbox.maxX(), local_bbox.maxY(), local_bbox.maxZ()}};
          local_kdtree_ptr_->Box_Search(search_box, local_pts);
          pcl::PointCloud<pcl::PointXYZ> pcd_pcl;
          pcd_pcl.points.assign(local_pts.begin(), local_pts.end());
          pcd_pcl.height = 1;
          pcd_pcl.width = pcd_pcl.points.size();
          // for (auto it : local_pts) pcd_pcl.push_back(it);
          sensor_msgs::PointCloud2 msg;
          pcl::toROSMsg(pcd_pcl, msg);
          msg.header.frame_id = map_frame_;
          msg.header.stamp = ros::Time::now();
          local_pcd_publisher_.publish(msg);
          d = ros::WallTime::now() - t1;
          std::cout << "<insertLocalCloudThread>: publish timecost: "
                    << d.toSec() << "s, map pts: " << local_pts.size()
                    << std::endl;
        }
      }

      publishLocalUpdateBBox();
      local_cloud_list_.pop_front();
    }
    rate.sleep();
  }
  std::cout << "<insertLocalCloudThread>: thread exits." << std::endl;
}  // namespace mapping_module

void WorldRepresentation::insertGlobalCloudThread() {
  ros::Rate rate(100);
  while (ros::ok() && !checkCancel()) {
    int list_size = 0;
    {  // lock 作用域
      std::lock_guard<std::mutex> lock(global_cloud_list_mutex_);
      list_size = global_cloud_list_.size();
    }
    if (list_size != 0) {
      std::lock_guard<std::mutex> lock_octree(global_octree_mutex_);
      std::lock_guard<std::mutex> lock(global_cloud_list_mutex_);
      if (global_cloud_list_.size() == 0)
        continue;  // 2nd check, avoid &p to be null
      while (global_cloud_list_.size() > max_buffer_size_)
        global_cloud_list_.pop_front();

      ros::WallTime t1 = ros::WallTime::now();
      input_pc_bbox_.reset();
      const std::pair<octomap::Pointcloud, octomap::point3d> &p =
          global_cloud_list_.front();

      // 全局更新时，给定空包围框，返回当前帧的包围框
      global_update_voxels_ = global_octree_ptr_->insertPointCloud(
          p.first, p.second, input_pc_bbox_, global_update_free_,
          lidar_range_max_);
      global_update_position_ = p.second;
      publishGlobalUpdateBBox();
      ros::WallDuration d = ros::WallTime::now() - t1;
      std::cout << "<insertGlobalCloudThread>: pts: " << p.first.size()
                << ", timecost: " << d.toSec() << "s" << std::endl;
      global_cloud_list_.pop_front();

      if (enable_terrain_) {  // 缓存到list中，用于地形分析
        std::lock_guard<std::mutex> lock_update(global_update_voxels_mutex_);
        global_update_voxels_list_.push_back(global_update_voxels_);
        global_update_position_list_.push_back(global_update_position_);
      }
    }
    rate.sleep();
  }
  std::cout << "<insertGlobalCloudThread>: thread exits." << std::endl;
}

void WorldRepresentation::publishLocalUpdateBBox() {
  if (!local_bbox_.isReset()) {
    visualization_msgs::Marker bbox_marker =
        rviz_utils::drawBoundingbox(local_bbox_);
    bbox_marker.header.frame_id = map_frame_;
    bbox_marker.header.stamp = ros::Time::now();
    local_bbox_publisher_.publish(bbox_marker);

    bbox_marker = rviz_utils::convertBoundingboxMsg(local_bbox_);
    bbox_marker.header.frame_id = map_frame_;
    bbox_marker.header.stamp = ros::Time::now();
    local_bbox_param_publisher_.publish(bbox_marker);
  }
}

void WorldRepresentation::publishGlobalUpdateBBox() {
  if (!input_pc_bbox_.isReset()) {
    visualization_msgs::Marker bbox_marker =
        rviz_utils::drawBoundingbox(input_pc_bbox_);
    bbox_marker.header.frame_id = map_frame_;
    bbox_marker.header.stamp = ros::Time::now();
    input_bbox_publisher_.publish(bbox_marker);

    bbox_marker = rviz_utils::convertBoundingboxMsg(input_pc_bbox_);
    bbox_marker.header.frame_id = map_frame_;
    bbox_marker.header.stamp = ros::Time::now();

    geometry_msgs::Point curr_point;
    curr_point.x = global_update_position_.x();
    curr_point.y = global_update_position_.y();
    curr_point.z = global_update_position_.z();
    bbox_marker.points.push_back(curr_point);

    input_bbox_param_publisher_.publish(bbox_marker);  // 发布参数

    // pcl::PointCloud<pcl::PointXYZ> voxel_keys_pc;
    // for (octomap::KeySet::iterator it = global_update_voxels_.first.begin();
    //      it != global_update_voxels_.first.end(); ++it) {
    //   octomap::OcTreeKey iter_key = *it;
    //   pcl::PointXYZ p(it->k[0], it->k[1], it->k[2]);
    //   voxel_keys_pc.push_back(p);
    // }
    // if (voxel_keys_pc.size() > 0) {
    //   sensor_msgs::PointCloud2 msg;
    //   pcl::toROSMsg(voxel_keys_pc, msg);
    //   msg.header.frame_id = map_frame_;
    //   msg.header.stamp = ros::Time::now();
    //   global_update_pcd_publisher_.publish(msg);
    // }
  }
}

bool WorldRepresentation::saveOctomapServerCallback(
    save_octomap::Request &request, save_octomap::Response &response) {
  std::string file_path = request.file_path;
  std::cout << "<saveOctomapServerCallback>: path " << file_path << std::endl;

  if (!boost::filesystem::is_directory(file_path)) {
    std::cout << "<saveOctomapServerCallback>: path not "
                 "exist, return false."
              << std::endl;
    return false;
  }

  std::string file_name = stampToDateString(ros::WallTime::now());

  if (local_octree_enable_) {
    std::string local_octo_path = utils::PathJoin(file_path, "local_" +  file_name + ".bt");
    std::lock_guard<std::mutex> lock_local(local_octree_mutex_);
    local_octree_ptr_->write(local_octo_path);
    std::cout << local_octo_path << std::endl;

    std::string local_pcd_path = utils::PathJoin(file_path, "local_" +  file_name + ".pcd");
    savePoints(local_pcd_path);
  }

  if (global_octree_enable_) {
    std::string global_octo_path =
        utils::PathJoin(file_path, "global_octree.bt");
    std::lock_guard<std::mutex> lock_global(global_octree_mutex_);
    global_octree_ptr_->write(global_octo_path);
    std::cout << global_octo_path << std::endl;

    if (global_octree_std_enable_) {
      std::string global_std_octo_path =
          utils::PathJoin(file_path, "global_octree_std.bt");
      global_octree_std_ptr_->write(global_std_octo_path);
    }
  }
  response.file_name = file_name;
  response.success = true;
  return true;
}

bool WorldRepresentation::queryBboxPCDServerCallback(
    query_bbox_pcd::Request &request, query_bbox_pcd::Response &response) {
  geometry_msgs::Point min = request.min_point;
  geometry_msgs::Point max = request.max_point;

  printf(
      "<saveOctomapServerCallback>: min(%.3f,%.3f,%.3f), "
      "max(%.3f,%.3f,%.3f). "
      "\n",
      min.x, min.y, min.z, max.x, max.y, max.z);
  pcl::PointCloud<pcl::PointXYZ> pcd;
  std::shared_ptr<octomap::IgTree> octree_tmp_;
  if (global_octree_enable_) {
    {
      std::lock_guard<std::mutex> lock_global(global_octree_mutex_);
      octree_tmp_ = global_octree_ptr_->deepClone();
    }

    auto start =
        octree_tmp_->begin_leafs_bbx(octomap::point3d(min.x, min.y, min.z),
                                     octomap::point3d(max.x, max.y, max.z));
    auto end = octree_tmp_->end_leafs_bbx();
    for (auto it = start; it != end; it++) {
      auto key = it.getKey();
      auto node = octree_tmp_->search(key);
      if (node && octree_tmp_->isNodeOccupied(node)) {
        auto p3d = octree_tmp_->keyToCoord(key);
        pcd.push_back(pcl::PointXYZ(p3d.x(), p3d.y(), p3d.z()));
      }
      // pcl::io::savePCDFileBinary("",pcd);
      // 存入点云，转为sensor_msgs
    }
  }
  if (pcd.size() > 0) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pcd, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time::now();
    response.pcd_in_bbox = msg;
    printf("<saveOctomapServerCallback>: get %d points\n",
           static_cast<int>(pcd.size()));
    return true;
  }

  ROS_ERROR("<saveOctomapServerCallback>: get no points");
  return false;
}

bool WorldRepresentation::saveFrame(const pcl::PointCloud<pcl::PointXYZ> &pcd,
                                    std::string file_name) {
  pcl::io::savePCDFile(file_name, pcd, true);
  std::cout << "<saveFrame>: ";
  return true;
}

void WorldRepresentation::regenerateGlobalOctomap(
    std::vector<Record> &new_records) {
  ros::WallTime t1 = ros::WallTime::now();
  resetGlobalOctreePtr();
  global_cloud_list_.clear();

  for (int i = 0; i < new_records.size(); i++) {
    auto &rec = new_records[i];
    Eigen::Isometry3d lidar_pose = rec.pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr frame(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr valid_frame(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_frame(
        new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile(rec.file, *frame);
    filterInvalidPoints(frame, valid_frame);

    Eigen::Quaterniond q(lidar_pose.rotation());
    Eigen::Vector3d p = lidar_pose.translation();
    std::cout << "<regenerateGlobalOctomap>: p: " << p.transpose() << ", q: "
              << q.coeffs().transpose()
              // << ", points0: " << frame->size()
              // << ", points1: " << valid_frame->size()
              // << ", nodes: " << global_octree_ptr_->getNumLeafNodes()
              << std::endl;
    if (!math_utils::isValidFloat(p.x()) || !math_utils::isValidFloat(p.y()) ||
        !math_utils::isValidFloat(p.z()) || !math_utils::isValidFloat(q.x()) ||
        !math_utils::isValidFloat(q.y()) || !math_utils::isValidFloat(q.z()) ||
        !math_utils::isValidFloat(q.w()))
      continue;
    if (!ros::ok() || checkCancel()) {
      std::cout << "<regenerateGlobalOctomap>: cancel. " << std::endl;
      break;
    }
    pcl::transformPointCloud(*valid_frame, *map_frame, lidar_pose.matrix());
    octomap::Pointcloud cloud_octomap;
    octomap::pclToOctomap(*map_frame, &cloud_octomap);

    octomap::Boundingbox global_bbox;  // isReset(true)
    global_bbox.reset();

    octomap::point3d lidar_origin(lidar_pose.translation().x(),
                                  lidar_pose.translation().y(),
                                  lidar_pose.translation().z());

    global_octree_ptr_->insertPointCloud(cloud_octomap, lidar_origin,
                                         global_bbox, global_update_free_,
                                         lidar_range_max_);
  }
  ros::WallDuration d = ros::WallTime::now() - t1;
  std::cout << "<regenerateGlobalOctomap>: insert " << new_records.size()
            << " frames, timecost " << d.toSec() << " s." << std::endl;
}
void WorldRepresentation::regenerateGlobalOctomap(
    Eigen::Isometry3d &curr_pose, pcl::PointCloud<pcl::PointXYZ> &global_map) {
  ros::WallTime t1 = ros::WallTime::now();
  resetGlobalOctreePtr();
  global_cloud_list_.clear();
  pcl::PointCloud<pcl::PointXYZ>::Ptr valid_frame(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (auto p : global_map.points) {
    if (isValidPoint(p)) valid_frame->push_back(p);
  }

  octomap::Pointcloud cloud_octomap;
  octomap::pclToOctomap(*valid_frame, &cloud_octomap);

  octomap::Boundingbox global_bbox;  // isReset(true)
  global_bbox.reset();

  octomap::point3d lidar_origin(curr_pose.translation().x(),
                                curr_pose.translation().y(),
                                curr_pose.translation().z());

  global_octree_ptr_->insertPointCloud(
      cloud_octomap, lidar_origin, global_bbox,
      0,     // global_update_free_: no free
      1e6);  // lidar_range_max_: keep all points

  ros::WallDuration d = ros::WallTime::now() - t1;
  std::cout << "<regenerateGlobalOctomap>: insert " << valid_frame->size()
            << " points, timecost " << d.toSec() << " s." << std::endl;
}

void WorldRepresentation::regenerateLocalOctomap(
    Eigen::Isometry3d &curr_pose, std::vector<Record> &new_records) {
  ros::WallTime t1 = ros::WallTime::now();
  resetLocalOctreePtr();
  local_cloud_list_.clear();

  std::vector<Record> neigh_records;
  // 找到邻域r的帧，重新生成local map
  for (auto rec : new_records) {
    Eigen::Vector3d dis = rec.pose.translation() - curr_pose.translation();
    float dz = dis.z();
    dis.z() = 0;
    if (dis.norm() < local_map_size_ / 2 && dz < local_map_height_ / 2) {
      neigh_records.push_back(rec);
    }
  }

  for (auto rec : neigh_records) {
    Eigen::Isometry3d lidar_pose = rec.pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr frame(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr valid_frame(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_frame(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(rec.file, *frame);
    filterInvalidPoints(frame, valid_frame);

    Eigen::Quaterniond q(lidar_pose.rotation());
    Eigen::Vector3d p = lidar_pose.translation();
    std::cout << "<regenerateLocalOctomap>: p: " << p.transpose() << ", q: "
              << q.coeffs().transpose()
              // << ", points0: " << frame->size()
              // << ", points1: " << valid_frame->size()
              // << ", nodes: " << local_octree_ptr_->getNumLeafNodes()
              << std::endl;
    if (!math_utils::isValidFloat(p.x()) || !math_utils::isValidFloat(p.y()) ||
        !math_utils::isValidFloat(p.z()) || !math_utils::isValidFloat(q.x()) ||
        !math_utils::isValidFloat(q.y()) || !math_utils::isValidFloat(q.z()) ||
        !math_utils::isValidFloat(q.w()))
      continue;
    if (!ros::ok() || checkCancel()) {
      std::cout << "<regenerateLocalOctomap>: cancel. " << std::endl;
      break;
    }
    pcl::transformPointCloud(*valid_frame, *map_frame, lidar_pose.matrix());

    octomap::Boundingbox local_bbox;
    local_bbox.insertPoint(octomap::point3d(
        local_map_size_ / 2.0 + curr_pose.translation().x(),
        local_map_size_ / 2.0 + curr_pose.translation().y(),
        local_map_height_ / 2.0 + curr_pose.translation().z()));
    local_bbox.insertPoint(octomap::point3d(
        -local_map_size_ / 2.0 + curr_pose.translation().x(),
        -local_map_size_ / 2.0 + curr_pose.translation().y(),
        -local_map_height_ / 2.0 + curr_pose.translation().z()));
    // 去掉不在框里的
    valid_frame->clear();
    for (auto p : map_frame->points) {
      if (local_bbox.ifContain(octomap::point3d(p.x, p.y, p.z)))
        valid_frame->push_back(p);
    }

    octomap::point3d lidar_origin(lidar_pose.translation().x(),
                                  lidar_pose.translation().y(),
                                  lidar_pose.translation().z());
    octomap::Pointcloud cloud_octomap;
    octomap::pclToOctomap(*valid_frame, &cloud_octomap);
    local_octree_ptr_->insertPointCloud(cloud_octomap, lidar_origin, local_bbox,
                                        local_update_free_, lidar_range_max_);
  }
  ros::WallDuration d = ros::WallTime::now() - t1;
  std::cout << "<regenerateLocalOctomap>: insert " << neigh_records.size()
            << " frames, timecost " << d.toSec() << " s." << std::endl;
}

void WorldRepresentation::regenerateLocalOctomap(
    Eigen::Isometry3d &curr_pose, pcl::PointCloud<pcl::PointXYZ> &local_map) {
  ros::WallTime t1 = ros::WallTime::now();
  resetLocalOctreePtr();
  local_cloud_list_.clear();

  pcl::PointCloud<pcl::PointXYZ>::Ptr valid_frame(
      new pcl::PointCloud<pcl::PointXYZ>);

  octomap::Boundingbox local_bbox;
  local_bbox.insertPoint(
      octomap::point3d(local_map_size_ / 2.0 + curr_pose.translation().x(),
                       local_map_size_ / 2.0 + curr_pose.translation().y(),
                       local_map_height_ / 2.0 + curr_pose.translation().z()));
  local_bbox.insertPoint(
      octomap::point3d(-local_map_size_ / 2.0 + curr_pose.translation().x(),
                       -local_map_size_ / 2.0 + curr_pose.translation().y(),
                       -local_map_height_ / 2.0 + curr_pose.translation().z()));
  // 去掉不在框里的
  for (auto p : local_map.points) {
    if (isValidPoint(p) &&
        local_bbox.ifContain(octomap::point3d(p.x, p.y, p.z)))
      valid_frame->push_back(p);
  }

  octomap::point3d lidar_origin(curr_pose.translation().x(),
                                curr_pose.translation().y(),
                                curr_pose.translation().z());
  octomap::Pointcloud cloud_octomap;
  octomap::pclToOctomap(*valid_frame, &cloud_octomap);
  local_octree_ptr_->insertPointCloud(
      cloud_octomap, lidar_origin, local_bbox,
      0,     // local_update_free_: no free
      1e6);  // lidar_range_max_: keep all points

  ros::WallDuration d = ros::WallTime::now() - t1;
  std::cout << "<regenerateLocalOctomap>: insert " << valid_frame->size()
            << " points, timecost " << d.toSec() << " s." << std::endl;
}

bool WorldRepresentation::savePoints(std::string file_name) {
  pcl::PointCloud<pcl::PointXYZ> full_pc;
  for (auto it = local_octree_ptr_->begin_leafs();
       it != local_octree_ptr_->end_leafs(); ++it) {
    octomap::IgTreeNode *voxel = local_octree_ptr_->search(it.getKey());
    if (local_octree_ptr_->isNodeOccupied(voxel)) {
      pcl::PointXYZ p;
      p.x = voxel->mu().x();
      p.y = voxel->mu().y();
      p.z = voxel->mu().z();
      full_pc.push_back(p);
    }
  }

  std::cout << "<savePoints>: occupied: " << full_pc.size() << " voxels. "
            << std::endl;
  if (full_pc.size()  == 0) return false;
  pcl::io::savePCDFile(file_name, full_pc);
  return true;
}
};  // namespace mapping_module