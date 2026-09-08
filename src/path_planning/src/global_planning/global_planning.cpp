/* Copyright Year: 2026
 * Copyright Owner: Networked Robotics and Systems Lab
 * Authors: Yuxiang Li, Kun Chen, Haoyao Chen
 */
#include <astar_3d/astar_3d.h>  // astar头文件只能放在cpp中，放在.h中会报hash错误
#include <geometry_msgs/PoseArray.h>
#include <global_planning/global_planning.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Int8.h>
#define DEBUG_FLAG 0
GlobalPlanning::GlobalPlanning() {
  ros::NodeHandle nh;
  nh.getParam("global_planning_node/map_frame", map_frame_);
  nh.getParam("global_planning_node/base_footprint_frame",
              base_footprint_frame_);
  nh.getParam("global_planning_node/astar_hueristic_weight",
              astar_hueristic_weight_);
  nh.getParam("global_planning_node/astar_traversability_weight",
              astar_traversability_weight_);
  path_pub_ = nh.advertise<nav_msgs::Path>("global_planning/path", 1);
  end_points_pub_ = nh.advertise<visualization_msgs::Marker>(
      "global_planning/path_end_points", 1);
  path_pcd_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("global_planning/path_pcd", 1);
  pose_array_pub_ =
      nh.advertise<geometry_msgs::PoseArray>("global_planning/pose_array", 1);
  stop_pub_ = nh.advertise<std_msgs::Int8>("stop_cmd", 1);
  goal_sub_ = nh.subscribe("clicked_point", 1,
                           &GlobalPlanning::clickedPointCallback, this);
#if !DEBUG_FLAG  // DEBUG 模式下开启Timer读取tf
  // odom_timer_ = nh.createTimer(
  //     ros::Duration(0.1), &GlobalPlanning::odomTimerCallback, this);  //
  //     定时器
#endif
  octomap_sub_ = nh.subscribe("mapping_module/global_octree", 1,
                              &GlobalPlanning::octomapCallback, this);

  query_drivable_point_client_ =
      nh.serviceClient<mapping_module::query_nearest_drivable_point>(
          "/mapping_module/terrain_evaluation/query_nearest_drivable_point");
}

void GlobalPlanning::octomapCallback(const octomap_msgs::Octomap& msg) {
  octomap::IgTree temp_ig_tree(0.5);
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  if (aot) {
    if (global_octree_ == NULL) {
      global_octree_.reset(new octomap::IgTree(0.2));
    } else {
      global_octree_->clear();
    }
    global_octree_.reset(dynamic_cast<octomap::IgTree*>(aot));
    map_flag_ = map_flag_ > 1 ? 1 : map_flag_ + 1;
  }
}

// 定时读取tf
void GlobalPlanning::odomTimerCallback(const ros::TimerEvent& event) {
  if (getSensorPoseEigen(&base_pose_, ros::Time(0))) {
    geometry_msgs::Point p;
    p.x = base_pose_.translation().x();
    p.y = base_pose_.translation().y();
    p.z = base_pose_.translation().z();
    odom_flag_ = (checkPointDrivable(p, position_) > 0);
  }
}

// 有了map、odom、goal后，清掉map_flag_，等待新的map_flag_开始规划
void GlobalPlanning::clickedPointCallback(
    const geometry_msgs::PointStamped& cp_msg) {
  if (map_flag_ <= 0) return;
#if !DEBUG_FLAG  // 非 DEBUG 模式下，odom为起点，clicked_point提供终点
  if (!odom_flag_) return;

  octomap::OcTreeKey key = global_octree_->coordToKey(
      octomap::point3d(cp_msg.point.x, cp_msg.point.y, cp_msg.point.z));
  octomap::IgTreeNode* node = global_octree_->search(key);

  printf("<clickedPointCallback>: Goal = [%.1f, %.1f, %.1f]. ", cp_msg.point.x,
         cp_msg.point.y, cp_msg.point.z);
  if (node) {
    if (global_octree_->isNodeOccupied(node)) {
      std::cout << "Occupied voxel" << std::endl;
    } else {
      std::cout << "Free voxel" << std::endl;
    }
  } else {
    std::cout << "Unknown voxel" << std::endl;
  }

  std_msgs::Int8 stop_msg;
  stop_pub_.publish(stop_msg);

  goal_ = cp_msg.point;
  checkGoalReachable();

#else  // DEBUG 模式下，由clicked_point连续两次获得起点、终点
  octomap::OcTreeKey key = global_octree_->coordToKey(
      octomap::point3d(cp_msg.point.x, cp_msg.point.y, cp_msg.point.z - 0.1));
  octomap::IgTreeNode* node = global_octree_->search(key);
  printf(
      "<GlobalPlanning::clickedPointCallback>: Selected point = [%.1f, %.1f, "
      "%.1f]. ",
      cp_msg.point.x, cp_msg.point.y, cp_msg.point.z);
  if (node) {
    if (global_octree_->isNodeOccupied(node)) {
      std::cout << "Occupied voxel" << std::endl;
    } else {
      std::cout << "Free voxel -> Return." << std::endl;
      return;
    }
  } else {
    std::cout << "Unknown voxel -> Return." << std::endl;
    return;
  }

  static bool start_flag = false;
  if (!start_flag) {  // 还没有选第一个点
    start_flag = true;
    goal_flag_ = false;
    position_ = cp_msg.point;
  } else {  // 选了第一个点
    start_flag = false;
    goal_flag_ = true;
    goal_checked_ = cp_msg.point;
  }
#endif
}

nav_msgs::Path pathInterpolation(nav_msgs::Path path) {
  nav_msgs::Path path_int;
  if (path.poses.size() <= 1) return path;
  path_int.header = path.header;
  const int n = 20;  // 0.2 -> 0.025
  for (int i = 0; i < path.poses.size() - 1; i++) {
    Eigen::Vector3f p0(path.poses[i].pose.position.x,
                       path.poses[i].pose.position.y,
                       path.poses[i].pose.position.z);
    Eigen::Vector3f p1(path.poses[i + 1].pose.position.x,
                       path.poses[i + 1].pose.position.y,
                       path.poses[i + 1].pose.position.z);

    Eigen::Vector3f delta = p1 - p0;
    for (int t = 0; t < n; t++) {  // 2个点，插为n个点
      Eigen::Vector3f p = p0 + delta * t / n;
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      path_int.poses.push_back(pose);
      // printf(
      //     "<pathInterpolation>: wps[%02d][%02d]: "
      //     "%.3f,%.3f,%.3f\n",
      //     i, t, p[0], p[1], p[2]);
    }
  }
  return path_int;
}

void GlobalPlanning::publishPath(nav_msgs::Path path) {
  geometry_msgs::PoseArray pose_array;
  if (path.poses.size() <= 0) return;
  static int cnt = 0;
  path.header.frame_id = map_frame_;
  path.header.seq = cnt++;
  path.header.stamp = ros::Time::now();
  pose_array.header = path.header;

  geometry_msgs::Quaternion flipper_angle;
  flipper_angle.x = 0;
  flipper_angle.y = 0;
  flipper_angle.z = 0;
  flipper_angle.w = 0;
  for (int i = 0; i < path.poses.size(); i++) {
    pose_array.poses.push_back(path.poses[i].pose);
    pose_array.poses.back().orientation = flipper_angle;
    // path.poses[i].pose.position.z += 0.5;
  }
  path_pub_.publish(path);
  pose_array_pub_.publish(pose_array);

  pcl::PointCloud<pcl::PointXYZI> path_pcd;
  nav_msgs::Path path_dense = pathInterpolation(path);
  for (auto pose : path_dense.poses) {
    pcl::PointXYZI p(255);
    p.x = pose.pose.position.x;
    p.y = pose.pose.position.y;
    p.z = pose.pose.position.z + 0.5;
    path_pcd.push_back(p);
  }
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(path_pcd, msg);
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  path_pcd_pub_.publish(msg);
}

bool GlobalPlanning::getSensorPoseTF(tf::StampedTransform* pose,
                                     ros::Time timestamp) {
  static tf::TransformListener listener;
  try {
    // listener.waitForTransform(map_frame_, base_footprint_frame_,
    // ros::Time(0), ros::Duration(0.5));
    listener.lookupTransform(map_frame_, base_footprint_frame_, timestamp,
                             *pose);
  } catch (...) {
    ROS_ERROR("Listen TF [%.6f] (%s -> %s) timeout!", timestamp.toSec(),
              map_frame_.c_str(), base_footprint_frame_.c_str());
    return false;
  }
  return true;
}

bool GlobalPlanning::getSensorPoseEigen(Eigen::Isometry3d* pose,
                                        ros::Time timestamp) {
  tf::StampedTransform lidar_transform_tf;
  if (!getSensorPoseTF(&lidar_transform_tf, timestamp)) {
    return false;
  }
  tf::transformTFToEigen(lidar_transform_tf, *pose);

  return true;
}

void GlobalPlanning::visualizeEndpoints(PointPair point_pair) {
  visualization_msgs::Marker end_vis;

  end_vis.header.stamp = ros::Time::now();
  end_vis.header.frame_id = map_frame_;

  end_vis.ns = "end_points";
  end_vis.id = 0;
  end_vis.type = visualization_msgs::Marker::SPHERE_LIST;
  end_vis.action = visualization_msgs::Marker::ADD;
  end_vis.scale.x = 0.5;
  end_vis.scale.y = 0.5;
  end_vis.scale.z = 0.5;
  end_vis.pose.orientation.x = 0.0;
  end_vis.pose.orientation.y = 0.0;
  end_vis.pose.orientation.z = 0.0;
  end_vis.pose.orientation.w = 1.0;

  end_vis.color.a = 1.0;
  end_vis.color.r = 0.0;
  end_vis.color.g = 1.0;
  end_vis.color.b = 0.0;

  end_vis.points.clear();

  point_pair.first.z += 0.5;
  point_pair.second.z += 0.5;
  end_vis.points.push_back(point_pair.first);
  end_vis.points.push_back(point_pair.second);
  end_points_pub_.publish(end_vis);
}

// 2: drivable, 1: near drivable, 0: query failed
int GlobalPlanning::checkPointDrivable(geometry_msgs::Point p_in,
                                       geometry_msgs::Point& p_out) {
  mapping_module::query_nearest_drivable_point srv;
  srv.request.query_point = p_in;  // 查询最近邻
  if (!query_drivable_point_client_.call(srv)) {
    ROS_ERROR("<checkPointDrivable>: query_drivable_point_client failed");
    return 0;
  }

  p_out = srv.response.result_point;
  if (srv.response.success) {
    return 2;
  }

  // printf("<checkPointDrivable>: [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f].\n",
  //        p_in.x, p_in.y, p_in.z, p_out.x, p_out.y, p_out.z);
  return 1;
}

int GlobalPlanning::checkGoalReachable() {
  int ret = checkPointDrivable(goal_, goal_checked_);
  if (ret > 0) {
    goal_flag_ = true;
    map_flag_ = -1;  // 等待下次的地图到来
    // if (ret == 2) goal_reachable_flag_ = true;  // unused
  }
  return ret;
}

bool GlobalPlanning::checkEndPoints(PointPair in, PointPair out) {
  if (checkPointDrivable(in.first, out.first)) return false;
  if (checkPointDrivable(in.second, out.second)) return false;
  return true;
}

bool GlobalPlanning::search(nav_msgs::Path& path) {
  if (getSensorPoseEigen(&base_pose_, ros::Time(0))) {
    geometry_msgs::Point p;
    p.x = base_pose_.translation().x();
    p.y = base_pose_.translation().y();
    p.z = base_pose_.translation().z();
    odom_flag_ = (checkPointDrivable(p, position_) > 0);
  } else {
    ROS_ERROR("<GlobalPlanning::search>: getSensorPoseEigen failed.");
    return false;
  }
  if (odom_flag_ && goal_flag_) {
    Eigen::Vector3f dis = Eigen::Vector3f(
        goal_.x - position_.x, goal_.y - position_.y, goal_.z - position_.z);
    if (dis.norm() <= 0.2) {  // 已经到达Goal
      goal_flag_ = false;
      last_plan_state_ = -1;  // reset flag
      printf("<GlobalPlanning::search>: Goal has reached.");
      std_msgs::Int8 stop_msg;
      stop_pub_.publish(stop_msg);
      return false;
    } else {
      checkPointDrivable(goal_, goal_checked_);  // 一直刷新目标点
    }
  } else {
    return false;
  }

#if !DEBUG_FLAG
  if (last_plan_state_ == 0) {   // 还没规划成功
    last_position_ = position_;  // 然后规划
    last_time_ = ros::Time::now();
  } else {  // 有了规划成功的路径，需要等odom移动足够距离，再规划
    Eigen::Vector3f dis(position_.x - last_position_.x,
                        position_.y - last_position_.y,
                        position_.z - last_position_.z);
    ros::Duration d = ros::Time::now() - last_time_;
    if (dis.norm() < 0.2 && d.toSec() < 2) return false;  // do nothing
    // dis > 0.2 || d.toSec() > 2 , replanning
  }
#endif
  if (goal_flag_ && map_flag_ > 0) {  // 有了新目标，则开始新规划
    path.poses.clear();
    bool ret = planPath(std::make_pair(position_, goal_checked_), path);

    if (ret) {
      last_position_ = position_;
      last_time_ = ros::Time::now();
      geometry_msgs::Point end_pt = path.poses.back().pose.position;
      float dis_xy =
          Eigen::Vector2f(end_pt.x - goal_.x, end_pt.y - goal_.y).norm();
      float dis_z = end_pt.z - goal_.z;
      printf(
          "<GlobalPlanning::search>: The end waypoint [%.3f,%.3f,%.3f], goal "
          "[%.3f,%.3f,%.3f], dis_xy=%.3f, dis_z=%.3f\n\n",
          end_pt.x, end_pt.y, end_pt.z, goal_checked_.x, goal_checked_.y,
          goal_checked_.z, dis_xy, dis_z);
#if !DEBUG_FLAG
      if (dis_xy < 0.2 && dis_z < 0.2) {  // 距离较近，则认为到达，停止规划
        last_plan_state_ = 2;
        // goal_flag_ = false;
      } else {  // 若未到达，重新找最近邻目标点，继续规划
        last_plan_state_ = 1;
        // checkPointDrivable(goal_, goal_checked_);
      }
#else  // DEBUG
      goal_flag_ = false;
#endif
    }
  }
  return true;
}

//  !goal_flag_ || replan_flag_
bool GlobalPlanning::planPath(PointPair endPoints, nav_msgs::Path& path) {
  int timeout = 10;
  while (path.poses.size() == 0 && timeout > 0) {
    Astar astar_3d(global_octree_.get(), astar_hueristic_weight_,
                   astar_traversability_weight_, 1);
    path.poses.clear();
    astar_3d.SetStartAndGoal(
        Eigen::Vector3f(endPoints.first.x, endPoints.first.y,
                        endPoints.first.z),
        Eigen::Vector3f(endPoints.second.x, endPoints.second.y,
                        endPoints.second.z));

    path = astar_3d.startSearch();
    timeout--;
  }

  publishPath(path);
  visualizeEndpoints(endPoints);
  return (path.poses.size() > 0);
}
