#ifndef __GLOBAL_PLANNING_H__
#define __GLOBAL_PLANNING_H__

#include <geometry_msgs/PointStamped.h>
#include <mapping_module/world_representation/world_representation.h>
#include <nav_msgs/Path.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <mapping_module/query_nearest_drivable_point.h>

using PointPair =
    std::pair<geometry_msgs::Point, geometry_msgs::Point>;  // origin, goal
class GlobalPlanning {
 public:
  GlobalPlanning();
  ~GlobalPlanning() {
    if (global_octree_) global_octree_->clear();
  }

  void octomapCallback(const octomap_msgs::Octomap& msg);
  void clickedPointCallback(const geometry_msgs::PointStamped& cp_msg);
  void visualizeEndpoints(PointPair point_pair);
  void publishPath(nav_msgs::Path path);
  bool getSensorPoseTF(tf::StampedTransform* pose, ros::Time timestamp);
  bool getSensorPoseEigen(Eigen::Isometry3d* pose, ros::Time timestamp);
  void odomTimerCallback(const ros::TimerEvent& event);
  bool planPath(PointPair endPoints, nav_msgs::Path& path);  // NOLINT
  bool checkEndPoints(PointPair in, PointPair out);
  int checkPointDrivable(geometry_msgs::Point p_in,
                         geometry_msgs::Point& p_out);  // NOLINT
  int checkGoalReachable();
  void run();
  bool search(nav_msgs::Path &path);  // NOLINT

  ros::Subscriber goal_sub_, octomap_sub_;
  ros::Publisher path_pub_, end_points_pub_, pose_array_pub_, path_pcd_pub_, stop_pub_;
  ros::Timer odom_timer_;
  ros::ServiceClient query_drivable_point_client_;

  std::shared_ptr<octomap::IgTree> global_octree_;  // NOLINT
  std::string base_footprint_frame_ = "base_link";
  std::string map_frame_ = "map";  // NOLINT

  int map_flag_ = 0;
  int last_plan_state_ = -1;  // 0: no goal and map, 1: path close to goal, 2: path achieves goal
  Eigen::Isometry3d base_pose_;
  geometry_msgs::Point goal_, goal_checked_;
  geometry_msgs::Point position_, last_position_;
  ros::Time last_time_;
  // PointPair endPointsChecked_;
  bool odom_flag_ = false;
  bool goal_flag_ = false;
  bool goal_reachable_flag_ = false;
  bool replan_flag_ = false;
  float astar_hueristic_weight_ = 0.5;
  float astar_traversability_weight_ = 0.5;
  // std::list<geometry_msgs::PoseStamped> path_list_;
};

#endif