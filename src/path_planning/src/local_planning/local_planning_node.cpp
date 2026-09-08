#include "local_planning/local_planning.h"

#define BACKWARD_HAS_BFD 1
#include <backward_ros/backward.hpp>
int main(int argc, char **argv) {
  backward::SignalHandling sh;
  ros::init(argc, argv, "local_planning_node");
  ros::NodeHandle nh;

  LocalPlanning local_planner;
  if (!local_planner.global_mode_) {
    std::cout
        << "<local_planning_node>: global_mode, start MultiThreadedSpinner."
        << std::endl;
    ros::MultiThreadedSpinner spinner(9);
    spinner.spin();
  } else {
    std::cout << "<local_planning_node>: !global_mode, start SingleSpinner."
              << std::endl;
    while (ros::ok()) {
      local_planner.Run();
      ros::spinOnce();
      usleep(1000);
    }
  }
}
