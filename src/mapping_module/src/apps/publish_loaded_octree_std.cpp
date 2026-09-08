#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

#include <iostream>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "publish_loaded_octree_std_node");
  ros::NodeHandle nh;
  ros::Publisher octomap_pub =
      nh.advertise<octomap_msgs::Octomap>("std_octomap", 1);

  std::string gt_tree_path =
      "/home/xxx/data/corridor/"
      "global_octree_std.bt";
  octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(gt_tree_path);
  octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
  std::cout << "path = " << gt_tree_path << std::endl;

  octomap_msgs::Octomap octomap_msg;
  octomap_msgs::fullMapToMsg(*octree, octomap_msg);
  octomap_msg.header.frame_id = "map";
  ros::Rate rate(1);
  while (ros::ok()) {
    octomap_msg.header.stamp = ros::Time::now();
    octomap_pub.publish(octomap_msg);
    std::cout << "publish once" << std::endl;
    // rate.sleep();
    sleep(1);
  }
}