#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

#include <iostream>
#define BACKWARD_HAS_BFD 1
#include <backward_ros/backward.hpp>

#include "mapping_module/world_representation/world_representation.h"

octomap::IgTree temp_ig_tree(  // To avoid error: "Could not create octree of
    0.5);  // type IgTree, not in store in classIDMapping..."
std::shared_ptr<octomap::IgTree> octree_ptr_;

int main(int argc, char* argv[]) {
  backward::SignalHandling sh;
  ros::init(argc, argv, "publish_loaded_octree_node");
  ros::NodeHandle nh;

  std::string octo_topic = "/mapping_module/global_octree";
  std::string octo_path = "/home/xxx/tmp/test_in.bt";
  std::string map_frame = "map";
  nh.getParam("publish_loaded_octree/octo_topic", octo_topic);
  nh.getParam("publish_loaded_octree/octo_path", octo_path);
  nh.getParam("map_frame", map_frame);

  ros::Publisher octomap_pub =
      nh.advertise<octomap_msgs::Octomap>(octo_topic, 1);
  octree_ptr_.reset(
      dynamic_cast<octomap::IgTree*>(octree_ptr_->read(octo_path)));

  std::cout << "Params" << std::endl
            << "load_path = " << octo_path << std::endl
            << "map_frame = " << map_frame << std::endl
            << "publish_topic = " << octo_topic << std::endl;

  ros::Rate rate(1);
  while (ros::ok()) {
    ros::WallTime t1 = ros::WallTime::now();
    std::shared_ptr<octomap::IgTree> temp_tree = octree_ptr_->deepClone();
    ros::WallDuration d = ros::WallTime::now() - t1;
    std::cout << "Clone timecost = " << d.toSec()
              << ", memoryFullGrid = " << temp_tree->memoryFullGrid()
              << ", memoryUsage = " << temp_tree->memoryUsage()
              << ", memoryUsageNode = " << temp_tree->memoryUsageNode()
              << std::endl;

    ros::WallTime t2 = ros::WallTime::now();
    auto begin = octree_ptr_->begin_leafs();
    auto end = octree_ptr_->end_leafs();
    int cnt = 0;
    for (auto it = begin; it != end && ros::ok(); it++) {
      auto key1 = it.getKey();
      // auto p1 = octree_ptr_->keyToCoord(key1);
      auto node1 = octree_ptr_->search(key1);

      // auto key2 = temp_tree->coordToKey(p1);
      // auto node2 = temp_tree->search(key2);
      if (node1 && octree_ptr_->isNodeOccupied(node1)) {  //
        //  && (node2 && temp_tree->isNodeOccupied(node2))) {
        // octree_ptr_->updateNode(p1, false);
        // 修改octree_ptr_，但temp_tree也会发生变化，说明不是深拷贝
        cnt++;
      }
    }
    d = ros::WallTime::now() - t2;
    std::cout << "Ergodic over timecost = " << d.toSec() << ", cnt = " << cnt
              << ", memoryFullGrid = " << octree_ptr_->memoryFullGrid()
              << ", tmemoryUsage = " << octree_ptr_->memoryUsage()
              << ", memoryUsageNode = " << octree_ptr_->memoryUsageNode()
              << std::endl;

    ros::WallTime t3 = ros::WallTime::now();
    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.stamp = ros::Time::now();
    octomap_msgs::fullMapToMsg(*temp_tree, octomap_msg);
    octomap_msg.header.frame_id = map_frame;
    octomap_pub.publish(octomap_msg);
    d = ros::WallTime::now() - t3;
    std::cout << "Publish timecost = " << d.toSec() << std::endl;
    // break;
    // octomap_msgs::Octomap octomap_msg;
    // octomap_msg.header.stamp = ros::Time::now();
    // octomap_msgs::fullMapToMsg(*octree_ptr_, octomap_msg);
    // octomap_msg.header.frame_id = map_frame;
    // octomap_pub.publish(octomap_msg);
    // std::cout << "publish once" << std::endl;
    rate.sleep();
  }
}