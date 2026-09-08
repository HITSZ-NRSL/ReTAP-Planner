#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

#include "mapping_module/world_representation/world_representation.h"

octomap::IgTree temp_ig_tree(0.5);
std::shared_ptr<octomap::IgTree> local_octree_;
std::shared_ptr<octomap::IgTree> global_octree_;

void loadLocalOctomap(std::string octree_path) {
  if (!local_octree_) local_octree_.reset(new octomap::IgTree(0.2));
  local_octree_.reset(
      dynamic_cast<octomap::IgTree*>(local_octree_->read(octree_path)));
  ROS_INFO("<loadLocalOctomap>: %s", octree_path.c_str());
}

void loadGlobalOctomap(std::string octree_path) {
  if (!global_octree_) global_octree_.reset(new octomap::IgTree(0.2));
  global_octree_.reset(
      dynamic_cast<octomap::IgTree*>(global_octree_->read(octree_path)));
  ROS_INFO("<loadGlobalOctomap>: %s", octree_path.c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_publisher_node");
  ros::NodeHandle nh;

  ros::Publisher local_map_pub =
      nh.advertise<octomap_msgs::Octomap>("mapping_module/local_octree", 1);
  ros::Publisher global_map_pub =
      nh.advertise<octomap_msgs::Octomap>("mapping_module/global_octree", 1);

  std::string local_map_path =
      "/home/liyx/gitlab-related/modules-mapping/data/data1/1697701706.8931269646.bt";
  // "2023-08-27-stairs-local_octree.bt";
  std::string global_map_path =
      "/home/liyx/gitlab-related/modules-mapping/data/data1/1697701706.8931269646.bt";
  // "2023-08-27-stairs_global_octree.bt";
  nh.getParam("local_map_path", local_map_path);
  nh.getParam("global_map_path", global_map_path);
  loadLocalOctomap(local_map_path);
  loadGlobalOctomap(global_map_path);

  octomap_msgs::Octomap local_octomap_msg, global_octomap_msg;
  octomap_msgs::fullMapToMsg(*local_octree_, local_octomap_msg);
  octomap_msgs::fullMapToMsg(*global_octree_, global_octomap_msg);
  local_octomap_msg.header.frame_id = "map";
  global_octomap_msg.header.frame_id = "map";
  octomap::IgTree* tmp = global_octree_.get();
  ros::Rate loop(0.5);
  int cnt = 0;
  while (ros::ok()) {
    local_octomap_msg.header.stamp = ros::Time::now();
    global_octomap_msg.header.stamp = ros::Time::now();
    local_map_pub.publish(local_octomap_msg);
    global_map_pub.publish(global_octomap_msg);
    uint64_t addr = (uint64_t)tmp;
    ROS_INFO("<map_publisher>: publish once.");
    loop.sleep();
  }
}
