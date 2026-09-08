#include <ros/ros.h>

#include "mapping_module/send_keyframe_data.h"
#include "mapping_module/send_map_data.h"

bool sendKeyframeDataServerCallback(
    mapping_module::send_keyframe_data::Request &request,      // NOLINT
    mapping_module::send_keyframe_data::Response &response) {  // NOLINT
  response.data_path =
      "/home/xxx/KeyFrame.txt";
  return true;
}

bool sendMapDataServerCallback(
    mapping_module::send_map_data::Request &request,      // NOLINT
    mapping_module::send_map_data::Response &response) {  // NOLINT
  response.LocalMap_path =
      "/home/xxx/LocalMap.pcd";
  response.GlobalMap_path =
      "/home/xxx/GlobalMap.pcd";
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "send_relo_data_node");
  ros::NodeHandle nh;
  ros::ServiceServer server1_ = nh.advertiseService(
      "send_relo_keyframe_data", &sendKeyframeDataServerCallback);
  ros::ServiceServer server2_ =
      nh.advertiseService("send_relo_map_data", &sendMapDataServerCallback);
  ros::spin();
}