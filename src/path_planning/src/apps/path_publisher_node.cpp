#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

ros::Publisher pose_array_publisher_;
void PublishLocalPath() {
  geometry_msgs::PoseArray pose_array;
  static int k = 0;
  for (int i = 0; i < 10; i++) {
    geometry_msgs::PoseStamped p;
    p.pose.position.x = k * 0.1;
    p.pose.position.y = 0;
    p.pose.position.z = 0;

    p.pose.orientation.x = (k - 5) * 0.1;
    p.pose.orientation.y = (k - 5) * 0.1;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 0;

    pose_array.poses.push_back(p.pose);
  }
  if (k ++ > 10) {
    k  = 0;
  }

  pose_array.header.frame_id = "map";
  pose_array.header.stamp = ros::Time::now();
  pose_array_publisher_.publish(pose_array);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_publisher_node");
  ros::NodeHandle nh_;
  pose_array_publisher_ = nh_.advertise<geometry_msgs::PoseArray>(
      "next_base_path", 1);

  ros::Rate rate(0.3);
  while (ros::ok()) {
    PublishLocalPath();
    rate.sleep();
  }
}