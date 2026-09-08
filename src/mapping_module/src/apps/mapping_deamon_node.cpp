
#include <ros/ros.h>
#include <std_msgs/Int32.h>

std::string loop_notify_topic, boot_script_path, kill_script_path;

void loopCallback(const std_msgs::Int32 &msg) {
  if (msg.data == 0) {
    // 重定位开始，建图中止一切动作
    system(kill_script_path.c_str());
  } else if (msg.data == 1) {
    // 重定位结束，启动所有节点
    system(kill_script_path.c_str());
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapping_deamon_node");
  ros::NodeHandle nh;

  nh.getParam("loop_notify_topic", loop_notify_topic);
  nh.getParam("boot_script_path", boot_script_path);
  nh.getParam("kill_script_path", kill_script_path);
  ros::Subscriber loop_closure_sub_ =
      nh.subscribe(loop_notify_topic, 1, loopCallback);
  ros::spin();
}
