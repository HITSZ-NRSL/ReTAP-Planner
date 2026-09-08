#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

#include "mapping_module/world_representation/world_representation.h"

// octomap::IgTree temp_ig_tree(0.5);
std::shared_ptr<octomap::IgTree> octree_ptr_;
bool newMsgFlag = false;

// void loadOctomap(std::string gt_tree_path) {
//   octree_ptr_ =
//   dynamic_cast<octomap::IgTree*>(octree_ptr_->read(gt_tree_path));
// }

void octomapCallback(const octomap_msgs::Octomap& msg) {
  ros::WallTime t1 = ros::WallTime::now();
  octomap::IgTree temp_ig_tree(  // To avoid error: "Could not create octree of
      0.5);  // type IgTree, not in store in classIDMapping..."

  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  if (aot) {
    octree_ptr_.reset(dynamic_cast<octomap::IgTree*>(aot));
  }
  ros::WallDuration d = ros::WallTime::now() - t1;
  std::cout << "msgToMap timecost = " << d.toSec() << std::endl;

  int occ_cnt = 0, free_cnt = 0;
  for (auto it = octree_ptr_->begin_leafs(); it != octree_ptr_->end_leafs();
       ++it) {
    octomap::IgTreeNode* node = octree_ptr_->search(it.getKey());
    if (node) {
      if (octree_ptr_->isNodeOccupied(node))
        occ_cnt++;
      else
        free_cnt++;
    }
  }
  std::cout << "get occ " << occ_cnt << ", free " << free_cnt << std::endl;

  newMsgFlag = true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "save_subscribed_octree_node");
  ros::NodeHandle nh;

  std::string octo_path = "/home/xxx/tmp/test_out.bt";
  std::string octo_topic = "/mapping_module/global_octree";

  nh.getParam("save_subscribed_octree/octo_topic", octo_topic);
  nh.getParam("save_subscribed_octree/octo_path", octo_path);

  ros::Subscriber octomap_sub_ = nh.subscribe(octo_topic, 1, &octomapCallback);

  std::cout << "Params" << std::endl
            << "save_path = " << octo_path << std::endl
            << "advertise_topic = " << octo_topic << std::endl;

  ros::Rate rate(10);
  while (ros::ok()) {
    if (newMsgFlag) {
      newMsgFlag = false;
      octree_ptr_->write(octo_path);
      std::cout << "write once" << std::endl;
    }
    ros::spinOnce();
    rate.sleep();
  }
}