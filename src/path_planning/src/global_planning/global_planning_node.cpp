#include <global_planning/global_planning.h>
// #include <local_planning/trajectory_generator.h>
// #include <ros/ros.h>
#define BACKWARD_HAS_BFD 1
#include <backward_ros/backward.hpp>

int main(int argc, char** argv) {
  backward::SignalHandling sh;
  ros::init(argc, argv, "global_planning_node");
  ros::NodeHandle nh;
  GlobalPlanning global_planner;

  ros::Rate loop(1);
  while (ros::ok()) {
    nav_msgs::Path path;
    global_planner.search(path);
    usleep(1e5);
    ros::spinOnce();
  }
}
