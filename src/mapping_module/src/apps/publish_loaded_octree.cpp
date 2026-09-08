#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

#include <iostream>

#include "mapping_module/world_representation/world_representation.h"

octomap::IgTree temp_ig_tree(  // To avoid error: "Could not create octree of
    0.5);  // type IgTree, not in store in classIDMapping..."
octomap::IgTree* octree_ptr_;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "publish_loaded_octree_node");
  ros::NodeHandle nh;

  std::string octo_topic = "/mapping_module/global_octree";
  std::string octo_path = "/home/xxx/tmp/test_in.bt";
  std::string map_frame = "map";

  ros::Publisher octomap_pub =
      nh.advertise<octomap_msgs::Octomap>(octo_topic, 1);
  octree_ptr_ = dynamic_cast<octomap::IgTree*>(octree_ptr_->read(octo_path));

  std::cout << "Params" << std::endl
            << "save_path = " << octo_path << std::endl
            << "map_frame = " << map_frame << std::endl
            << "publish_topic = " << octo_topic << std::endl;

  octomap_msgs::Octomap octomap_msg;
  octomap_msgs::fullMapToMsg(*octree_ptr_, octomap_msg);
  octomap_msg.header.frame_id = "map";

  ros::Rate rate(1);
  while (ros::ok()) {
    octomap_msg.header.stamp = ros::Time::now();
    octomap_pub.publish(octomap_msg);
    std::cout << "publish once" << std::endl;
    rate.sleep();
  }
}