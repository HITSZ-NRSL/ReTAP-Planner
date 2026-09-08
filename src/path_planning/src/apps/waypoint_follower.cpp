/* Copyright Year: 2026
 * Copyright Owner: Networked Robotics and Systems Lab
 * Authors: Yuxiang Li, Kun Chen, Haoyao Chen
 */
 
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Dense>

class WaypointFollower {
 public:
  WaypointFollower();
  void OdomCallback(const nav_msgs::OdometryConstPtr msg);
  void PathCallback(const nav_msgs::PathConstPtr msg);
  void Run();
  bool getSensorPoseTF(tf::StampedTransform* pose, ros::Time timestamp);
  bool getSensorPoseEigen(Eigen::Isometry3d* pose, ros::Time timestamp);

 private:
  ros::Subscriber odom_subscriber_, path_subscriber_;
  ros::Publisher flipper_angle_publisher_;
  bool path_flag_ = false;
  bool odom_flag_ = false;
  std::string map_frame_ = "map";
  std::string base_footprint_frame_ = "base_link";
  std::vector<Eigen::Vector4f> waypoints_;
  Eigen::Vector2f position_;
  ros::Timer odom_timer_;
  tf::TransformListener listener_;
};

WaypointFollower::WaypointFollower() {
  ros::NodeHandle nh_;
  flipper_angle_publisher_ =
      nh_.advertise<geometry_msgs::Twist>("/cmd_flipper", 1);
  path_subscriber_ = nh_.subscribe("local_planning/path_output", 1,
                                   &WaypointFollower::PathCallback, this);
}

bool WaypointFollower::getSensorPoseTF(tf::StampedTransform* pose,
                                       ros::Time timestamp) {
  try {
    // listener_.waitForTransform(map_frame_, base_footprint_frame_, timestamp,
    //                           ros::Duration(0.1));
    listener_.lookupTransform(map_frame_, base_footprint_frame_, timestamp,
                              *pose);
  } catch (...) {
    ROS_ERROR("Listen TF (%s -> %s) timeout!", map_frame_.c_str(),
              base_footprint_frame_.c_str());
    return false;
  }
  return true;
}

bool WaypointFollower::getSensorPoseEigen(Eigen::Isometry3d* pose,
                                          ros::Time timestamp) {
  tf::StampedTransform lidar_transform_tf;
  if (!getSensorPoseTF(&lidar_transform_tf, timestamp)) {
    return false;
  }
  tf::transformTFToEigen(lidar_transform_tf, *pose);

  return true;
}

void WaypointFollower::PathCallback(const nav_msgs::PathConstPtr msg) {
  if (msg->poses.size() <= 1) return;
  path_flag_ = true;
  std::vector<Eigen::Vector4f> wps;
  for (auto p : msg->poses) {  // 转为Eigen::Vector4f类型
    wps.push_back(Eigen::Vector4f(p.pose.position.x, p.pose.position.y,
                                  p.pose.orientation.x, p.pose.orientation.y));
  }
  // 对path进行插值
  int n = 10;
  waypoints_.clear();
  for (int i = 0; i < wps.size() - 1; i++) {
    Eigen::Vector4f delta = wps[i + 1] - wps[i];
    if (n == 1) {
      waypoints_.push_back(wps[i]);
      printf(
          "<WaypointFollower::PathCallback>: get wps[%02d]: "
          "%.3f,%.3f "
          "| %.3f,%.3f\n",
          i, wps[i][0], wps[i][1], wps[i][2], wps[i][3]);
    } else {
      for (int t = 0; t < n; t++) {  // 2个点，插为5个点
        Eigen::Vector4f p = wps[i] + delta * t / n;
        waypoints_.push_back(p);
        printf(
            "<WaypointFollower::PathCallback>: interpolated wps[%02d][%02d]: "
            "%.3f,%.3f "
            "| %.3f,%.3f\n",
            i, t, p[0], p[1], p[2], p[3]);
      }
    }
  }
  std::cout << std::endl;
}

void WaypointFollower::Run() {
  if (path_flag_) {
    Eigen::Isometry3d base_pose;
    if (!getSensorPoseEigen(&base_pose, ros::Time(0))) {
      return;
    }
    position_.x() = base_pose.translation().x();
    position_.y() = base_pose.translation().y();

    float min_dis = 1e9;
    int min_id = 0;
    Eigen::Vector2f closest_p;
    // 找到最近邻的waypoints_点，并发布该点的flipper角度
    for (int i = 0; i < waypoints_.size(); i++) {
      auto p = waypoints_[i];
      float dis = (position_ - Eigen::Vector2f(p.x(), p.y())).norm();
      if (dis < min_dis) {
        min_dis = dis;
        min_id = i;
        closest_p = Eigen::Vector2f(p[2], p[3]);  // 取出角度
      }
    }
    printf("<Run>: closet [%d], dis=%.3f, angle=%.3f,%.3f\n", min_id, min_dis,
           closest_p[0], closest_p[1]);
    geometry_msgs::Twist msg;
    float delta = 0;
    msg.linear.x = closest_p[0] + delta;
    msg.linear.y = closest_p[0] + delta;
    msg.angular.x = closest_p[1] + delta;
    msg.angular.y = closest_p[1] + delta;
    flipper_angle_publisher_.publish(msg);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_follwer_node");
  ros::NodeHandle nh;

  WaypointFollower wp_follwer;
  ros::Rate loop(100);
  while (ros::ok()) {
    wp_follwer.Run();
    usleep(100 * 1000);
    ros::spinOnce();
  }
}
