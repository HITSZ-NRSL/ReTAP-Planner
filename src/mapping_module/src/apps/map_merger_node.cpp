#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "mapping_module/world_representation/world_representation.h"

ros::Subscriber octomap_subscriber_, path_subscriber_;
std::shared_ptr<octomap::IgTree> octree_ptr_, local_octree_;

std::string base_footprint_frame_ = "base";  // NOLINT
std::string map_frame_ = "base_init";        // NOLINT

bool map_flag = false;
tf::StampedTransform trans_;

bool getSensorToFootprintTF(tf::StampedTransform *trans,
                            ros::Time timestamp = ros::Time(0)) {
  static tf::TransformListener listener;
  try {
    // listener.waitForTransform(base_footprint_frame_, map_frame_, timestamp,
    //                           ros::Duration(0));
    listener.lookupTransform(map_frame_, base_footprint_frame_, timestamp,
                             *trans);
    std::cout << "trans: " << trans->getOrigin().getX() << ", "
              << trans->getOrigin().getY() << ", " << trans->getOrigin().getZ()
              << std::endl;
  } catch (...) {
    ROS_ERROR("Listen TF [%.3f] (%s -> %s) timeout!", timestamp.toSec(),
              base_footprint_frame_.c_str(), map_frame_.c_str());
    return false;
  }
  return true;
}

void OctomapCallback(const octomap_msgs::Octomap &msg) {
  octomap::IgTree temp_ig_tree(0.5);
  octomap::AbstractOcTree *aot = octomap_msgs::msgToMap(msg);
  getSensorToFootprintTF(&trans_);
  octomap::point3d position(trans_.getOrigin().x(), trans_.getOrigin().y(),
                            trans_.getOrigin().z());
  octomap::Boundingbox bbox;
  float box_radius = 15;
  bbox.insertPoint(position +
                   octomap::point3d(box_radius, box_radius, box_radius));
  bbox.insertPoint(position -
                   octomap::point3d(box_radius, box_radius, box_radius));
  if (aot) {
    if (!octree_ptr_) octree_ptr_.reset(new octomap::IgTree(0.025));
    octree_ptr_.reset(dynamic_cast<octomap::IgTree *>(aot));
    // 转为点云
    // pcd_.clear();
    auto start = octree_ptr_->begin_leafs();
    auto end = octree_ptr_->end_leafs();
    for (auto iter = start; iter != end; iter++) {
      if (bbox.ifContain(
              octomap::point3d(iter.getX(), iter.getY(), iter.getZ())))
        local_octree_->updateNode(iter.getKey(), true);
    }
    map_flag = true;
  }
}

void SaveCallback(const std_msgs::String &msg) {
  msg.data;
  auto start = local_octree_->begin_leafs();
  auto end = local_octree_->end_leafs();
  pcl::PointCloud<pcl::PointXYZ> pcd;
  for (auto iter = start; iter != end; iter++) {
    pcd.push_back(pcl::PointXYZ(iter.getX(), iter.getY(), iter.getZ()));
  }
  std::cout << "save: " << msg.data << ", points: " << pcd.size() << std::endl;
  pcl::io::savePCDFile(msg.data, pcd, true);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_merger_node");
  ros::NodeHandle nh;
  octomap_subscriber_ =
      nh.subscribe("mapping_module/local_octree", 1, &OctomapCallback);
  path_subscriber_ = nh.subscribe("save_path", 1, &SaveCallback);
  ros::Publisher local_map_pub = nh.advertise<octomap_msgs::Octomap>(
      "mapping_module/local_octree_merged", 1);

  local_octree_.reset(new octomap::IgTree(0.025));

  while (!getSensorToFootprintTF(&trans_) && ros::ok()) {
    sleep(1);
  }

  ros::Rate loop(1);
  while (ros::ok()) {
    if (map_flag) {
      map_flag = false;
      octomap_msgs::Octomap local_octomap_msg;
      octomap_msgs::fullMapToMsg(*local_octree_, local_octomap_msg);
      local_octomap_msg.header.frame_id = map_frame_;
      local_octomap_msg.header.stamp = ros::Time::now();
      local_map_pub.publish(local_octomap_msg);
    }
    ros::spinOnce();
  }
}
