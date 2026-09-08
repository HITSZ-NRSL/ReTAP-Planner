/* Copyright Year: 2023
 * Copyright Owner: Liyx
 */
#pragma once
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "local_planning/configuration.h"
#include "local_planning/sample_set.h"
#include "mapping_module/query_terrain_attribute.h"
#include "mapping_module/world_representation/world_representation.h"
#include "path_planning/query_contact_configuration.h"

enum ContactType {
  StableHull = 0,
  SinglePoint = 1,
  TwoPoints = 2,
  SmallHull = 3,
  MiddleHull = 4
};

class NeighDomain {
 public:
  Eigen::Vector3f base_center;
  Eigen::Quaternionf base_dir;
  float base_radius;

  Eigen::Vector3f normal_vector;
  Eigen::Quaternionf normal_rotation;

  pcl::PointCloud<pcl::PointXYZ> decentralized_points;
};

class ElevationPoint {
 public:
  double x;
  double y;
  double z;

  size_t hash_key_value_2d;
  octomap::OcTreeKey k3d;
  octomap::OcTreeKey k2d;
  ElevationPoint() {}

  ElevationPoint(double _x, double _y, double _z, octomap::OcTreeKey key) {
    x = _x;
    y = _y;
    z = _z;
    k2d = k3d;
    k2d.k[2] = 0;
    hash_key_value_2d =
        static_cast<size_t>(key.k[0]) + 1447 * static_cast<size_t>(key.k[1]);
    //+ 345637 * static_cast<size_t>(key.k[2]); // 只用2D key
    k3d = key;  // 保存原始3D key
  }

  // 没有传入key，不具备find功能
  ElevationPoint(double _x, double _y, double _z) {
    x = _x;
    y = _y;
    z = _z;
  }

  // find()搜索时，用key匹配
  bool operator==(const ElevationPoint &other) const {
    return hash_key_value_2d == other.hash_key_value_2d;
  }

  ElevationPoint operator-(const ElevationPoint &other) {
    return ElevationPoint(x - other.x, y - other.y, 0);
  }
  ElevationPoint operator-() { return ElevationPoint(-x, -y, -z); }
};
/// Provides a hash function on Keys
class ElevationHash {
 public:
  size_t operator()(const ElevationPoint &p3d) const {
    return p3d.hash_key_value_2d;
  }
};

using ElevationPointSet =
    std::unordered_set<ElevationPoint, ElevationHash>;  // NOLINT

class TempVariableSet {
 public:
  Eigen::Quaternionf pose;  // (0,0,1)到法向量的旋转
  Eigen::Vector3f position;
  Eigen::Vector3f normal;
  float max_height_track;
  float diff_tolerance;
  float hull_area;  // ratio: hull area / track area
  pcl::PointCloud<pcl::PointXYZRGB> hull_pcd;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_contact_trans;
  ElevationPointSet elevation_map;
  Eigen::Vector3f track_center_trans;
  Eigen::Vector3f axis_trans;
  pcl::PointCloud<pcl::PointXYZRGB> hull_pcd_trans;
  pcl::PointCloud<pcl::PointXYZRGB> track_boundries_trans;

  TempVariableSet() {}
  TempVariableSet(
      Eigen::Quaternionf _pose, Eigen::Vector3f _position,
      Eigen::Vector3f _normal, float _max_height_track, float _diff_tolerance,
      float _hull_area,
      pcl::PointCloud<pcl::PointXYZRGB> &_hull_pcd,             // NOLINT
      pcl::PointCloud<pcl::PointXYZRGB> &_cloud_contact_trans,  // NOLINT
      ElevationPointSet &_elevation_map,                        // NOLINT
      Eigen::Vector3f _track_center_trans, Eigen::Vector3f _axis_trans,
      pcl::PointCloud<pcl::PointXYZRGB> &_hull_pcd_trans,           // NOLINT
      pcl::PointCloud<pcl::PointXYZRGB> &_track_boundries_trans) {  // NOLINT
    pose = _pose;
    position = _position;
    normal = _normal;
    max_height_track = _max_height_track;
    diff_tolerance = _diff_tolerance;
    hull_pcd = _hull_pcd;
    cloud_contact_trans = _cloud_contact_trans;
    elevation_map = _elevation_map;
    hull_area = _hull_area;
    track_center_trans = _track_center_trans;
    axis_trans = _axis_trans;
    hull_pcd_trans = _hull_pcd_trans;
    track_boundries_trans = _track_boundries_trans;
  }
};

class LocalPlanning {
 public:
  LocalPlanning();

  void LoadOctomapFromPCD(std::string input_file);
  void LoadOctomapFromBT(std::string input_file);

  void InitTrackSamples();

  void GoalDirCallback(const geometry_msgs::PoseStampedConstPtr msg);
  void GoalPosCallback(const geometry_msgs::PointStampedConstPtr msg);
  void OdomCallback(const nav_msgs::OdometryConstPtr msg);
  void LocalOctomapToElevationGrid();
  void GlobalPathCallback(const nav_msgs::PathConstPtr msg);
  void SinglePointPlanning(geometry_msgs::Point p, geometry_msgs::Quaternion q);
  bool getBasePoseTF(tf::StampedTransform *pose, ros::Time timestamp);
  bool getBasePoseEigen(Eigen::Isometry3d *pose, ros::Time timestamp);
  octomap::OcTreeKey decentralizeKey(octomap::OcTreeKey key,
                                     octomap::OcTreeKey center);
  octomap::OcTreeKey translateKey(octomap::OcTreeKey key,
                                  octomap::OcTreeKey center);
  int SearchTrackContactAndRotation(
      NeighDomain &neigh_domain,  // NOLINT
      SampleSet &track_samples,   // NOLINT
      float diff_tolerance, float rotate_step, Eigen::Quaternionf pre_rotation,
      Eigen::Quaternionf &rotation_delta,  // NOLINT
      TempVariableSet &variable_set);      // NOLINT

  int SearchFlipperAngle(
      TempVariableSet &variable_set,                    // NOLINT
      pcl::PointCloud<pcl::PointXYZ> &flipper_corners,  // NOLINT
      pcl::PointCloud<pcl::PointXYZ> &flipper_samples,  // NOLINT
      float flipper_length, Eigen::Quaternionf base_dir, bool enable,
      float *flipper_angle,
      pcl::PointCloud<pcl::PointXYZRGB> &flipper_corners_trans,  // NOLINT
      pcl::PointCloud<pcl::PointXYZRGB> &cloud_contact_trans);   // NOLINT

  int SearchFlipperContactAndRotation(
      std::vector<std::pair<TempVariableSet, int>>
          &variable_set_list,          // NOLINT
      SampleSet &track_samples,        // NOLINT
      ConfigurationPoint &way_point);  // NOLINT

  void Run();
  bool Search(nav_msgs::Path &path, float max_dis = 1e2,
              bool interpolate = true,
              std::vector<ConfigurationPoint> *wps_ptr = nullptr);
  bool GetClosestPoint(octomap::point3d curr_pos, float search_radius,
                       octomap::point3d &closest_pos);  // NOLINT
  float GetOrientationBetweenTwoWaypoints(geometry_msgs::Point p1,
                                          geometry_msgs::Point p2);

  bool EstimateNormalVector(Eigen::Vector3f goal_position, float base_radius,
                            NeighDomain &neigh_domain,  // NOLINT
                            SampleSet *track_samples = NULL,
                            Eigen::Vector3f *goal_normal = NULL);

  void ColorPointcloud(pcl::PointCloud<pcl::PointXYZ> &cloud_in,  // NOLINT
                       int r, int g, int b,
                       pcl::PointCloud<pcl::PointXYZRGB> &cloud_out);  // NOLINT

  void DecolorPointcloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud_in,  // NOLINT
                         pcl::PointCloud<pcl::PointXYZ> &cloud_out);   // NOLINT
  void PublishTrackBoundaryMarkers(
      pcl::PointCloud<pcl::PointXYZRGB> &boundaries_pcd);  // NOLINT

  void PublishFlipperBoundaryMarkers(
      pcl::PointCloud<pcl::PointXYZRGB> &boundaries_pcd);  // NOLINT

  void PublishConvexHullMarkers(
      pcl::PointCloud<pcl::PointXYZRGB> &hull_pcd);  // NOLINT

  void PublishConvexHullMarkers(
      ros::Publisher &pub,                          // NOLINT
      pcl::PointCloud<pcl::PointXYZRGB> &hull_pcd,  // NOLINT
      Eigen::Vector3f track_center);

  void ClearConvexHullMarkers();

  void PublishVectorPoseMarker(ros::Publisher &pub,      // NOLINT
                               Eigen::Vector3f pos,      // NOLINT
                               Eigen::Vector3f dir);     // NOLINT
  void PublishVectorPoseMarker(ros::Publisher &pub,      // NOLINT
                               Eigen::Vector3f pos,      // NOLINT
                               Eigen::Quaternionf dir);  // NOLINT
  void PublishColorPointcloud(
      ros::Publisher &pub,                      // NOLINT
      pcl::PointCloud<pcl::PointXYZRGB> &pcd);  // NOLINT

  void ClearVectorPoseMarker();

  void GenerateElevationMap(ElevationPointSet &elevation_map,        // NOLINT
                            pcl::PointCloud<pcl::PointXYZ> &cloud);  // NOLINT

  Eigen::Vector3f GetAxisWithSinglePointContact(
      Eigen::Vector3f cloud_point, Eigen::Vector3f track_center,
      Eigen::Vector3f gravity_transformed);
  Eigen::Vector3f GetAxisWithTwoPointContact(
      std::pair<Eigen::Vector3f, Eigen::Vector3f> contact_points,
      Eigen::Vector3f track_center, Eigen::Vector3f gravity_transformed);
  Eigen::Vector3f GetAxisWithClosetHullEdge(
      pcl::PointCloud<pcl::PointXYZRGB> &hull_pcd,  // NOLINT
      Eigen::Vector3f track_center);
  Eigen::Vector3f GetAxisWithHullEigenVector(
      pcl::PointCloud<pcl::PointXYZRGB> &hull_pcd,  // NOLINT
      Eigen::Vector3f track_center);

  void CheckTrackContact(Eigen::Quaternionf base_dir,
                         pcl::PointCloud<pcl::PointXYZRGB> &hull_pcd,  // NOLINT
                         bool *left_contact, bool *right_contact,
                         bool *front_contact, bool *rear_contact,
                         float *bbox_area);

  float GetCrossVal(Eigen::Vector3f center, Eigen::Vector3f p1,
                    Eigen::Vector3f p2) {
    // 不能用auto v1 = ...!!!!
    Eigen::Vector3f v1 = p1 - center;  // x1 = p1.x - c.x,  x2 = p2.x - c.x
    Eigen::Vector3f v2 = p2 - center;  // y1 = p1.y - c.y,  y2 = p2.y - c.y
    float z = v1.x() * v2.y() - v2.x() * v1.y();  // x1 * y2 - x2 * y1
    return z;
  }

  bool GetCrossDir(Eigen::Vector3f center, Eigen::Vector3f p1,
                   Eigen::Vector3f p2) {
    float z = GetCrossVal(center, p1, p2);
    return z > 0;
  }

  Eigen::Vector2f PointToLine2D(Eigen::Vector2f P, Eigen::Vector2f Ls,
                                Eigen::Vector2f Le) {
    float y1 = Ls.y(), y2 = Le.y(), x1 = Ls.x(), x2 = Le.x();
    float A = y1 - y2;
    float B = x2 - x1;
    float C = -A * x1 - B * y1;

    float x0 = P.x(), y0 = P.y();
    float m = A * y0 - B * x0;  // 垂线方程Bx-Ay+m=0
    float A1 = B, B1 = -A, C1 = m;
    float D = A * B1 - A1 * B;
    float x = (B * C1 - B1 * C) / D;
    float y = (A1 * C - A * C1) / D;
    // https://blog.csdn.net/hjxu2016/article/details/111594359
    // https://www.zhihu.com/question/381406535/answer/1095948349

    // ROS_INFO("<PointToLine2D>: %.3fx + %.3fy + %.3fz = 0", A, B, C);
    // ROS_INFO("<PointToLine2D>: intersection (%.3f, %.3f)", x, y);
    return Eigen::Vector2f(x, y);
  }

  Eigen::Vector3f toEulerAngle(const Eigen::Quaternionf &q);

  bool CheckEulerConstraint(Eigen::Quaternionf pose,
                            Eigen::Quaternionf base_dir,
                            Eigen::Vector3f &euler,  // NOLINT
                            float roll_th, float pitch_th);

  bool PlanConfiguration(ConfigurationPoint &way_point,
                         Eigen::Vector3f *goal_normal = NULL);

  void testGetMinRect(
      pcl::PointCloud<pcl::PointXYZRGB> &hull_pcd_no_yaw);  // NOLINT
  void LocalOctomapCallback(const octomap_msgs::Octomap &msg);
  void GlobalOctomapCallback(const octomap_msgs::Octomap &msg);
  void LocalPoseArrayCallback(const geometry_msgs::PoseArrayConstPtr msg);
  void PublishLocalPath(std::vector<ConfigurationPoint> &waypoints);
  bool queryContactConfigurationServerFunction(
      path_planning::query_contact_configuration::Request &request,
      path_planning::query_contact_configuration::Response &response);
  bool queryContactConfigurationServerCallback(
      path_planning::query_contact_configuration::Request &request,
      path_planning::query_contact_configuration::Response &response) {
    std::lock_guard<std::mutex> lock(local_octomap_mutex_);
    return queryContactConfigurationServerFunction(request, response);
  }
  bool queryContactConfigurationServer0Callback(
      path_planning::query_contact_configuration::Request &request,
      path_planning::query_contact_configuration::Response &response) {
    std::lock_guard<std::mutex> lock(service0_mutex_);
    return queryContactConfigurationServerFunction(request, response);
  }
  bool queryContactConfigurationServer1Callback(
      path_planning::query_contact_configuration::Request &request,
      path_planning::query_contact_configuration::Response &response) {
    std::lock_guard<std::mutex> lock(service1_mutex_);
    return queryContactConfigurationServerFunction(request, response);
  }
  bool queryContactConfigurationServer2Callback(
      path_planning::query_contact_configuration::Request &request,
      path_planning::query_contact_configuration::Response &response) {
    std::lock_guard<std::mutex> lock(service2_mutex_);
    return queryContactConfigurationServerFunction(request, response);
  }
  bool queryContactConfigurationServer3Callback(
      path_planning::query_contact_configuration::Request &request,
      path_planning::query_contact_configuration::Response &response) {
    std::lock_guard<std::mutex> lock(service3_mutex_);
    return queryContactConfigurationServerFunction(request, response);
  }
  bool queryContactConfigurationServer4Callback(
      path_planning::query_contact_configuration::Request &request,
      path_planning::query_contact_configuration::Response &response) {
    std::lock_guard<std::mutex> lock(service4_mutex_);
    return queryContactConfigurationServerFunction(request, response);
  }
  bool queryContactConfigurationServer5Callback(
      path_planning::query_contact_configuration::Request &request,
      path_planning::query_contact_configuration::Response &response) {
    std::lock_guard<std::mutex> lock(service5_mutex_);
    return queryContactConfigurationServerFunction(request, response);
  }
  bool queryContactConfigurationServer6Callback(
      path_planning::query_contact_configuration::Request &request,
      path_planning::query_contact_configuration::Response &response) {
    std::lock_guard<std::mutex> lock(service6_mutex_);
    return queryContactConfigurationServerFunction(request, response);
  }
  bool queryContactConfigurationServer7Callback(
      path_planning::query_contact_configuration::Request &request,
      path_planning::query_contact_configuration::Response &response) {
    std::lock_guard<std::mutex> lock(service7_mutex_);
    return queryContactConfigurationServerFunction(request, response);
  }
  bool queryContactConfigurationServer8Callback(
      path_planning::query_contact_configuration::Request &request,
      path_planning::query_contact_configuration::Response &response) {
    std::lock_guard<std::mutex> lock(service8_mutex_);
    return queryContactConfigurationServerFunction(request, response);
  }

  std::shared_ptr<octomap::IgTree> local_octree_ptr_, global_octree_ptr_;
  std::string map_frame_ = "map";
  std::string base_footprint_frame_ = "base_link";
  ros::Publisher goal_3d_publisher_, goal_norm_publisher_,
      track_contact_pcd_publisher_, track_boundry_publisher,
      track_convex_hull_publisher_, flipper_contact_pcd_publisher_,
      track_rotation_axis_publisher_, flipper_boundry_publisher,
      base_convex_hull_publisher_, flipper_angle_publisher_, path_publisher_,
      pose_array_publisher_, local_costmap_publisher_,
      local_elevation_publisher_, local_elevation_pcd_publisher_;
  ros::Subscriber local_octomap_subscriber_, global_octomap_subscriber_,
      goal_dir_subscriber_, goal_pos_subscriber_, odom_subscriber_,
      global_path_subscriber_, local_pose_array_subscrber_;
  ros::ServiceServer query_contact_configuration_server_,
      query_contact_configuration_server0_,
      query_contact_configuration_server1_,
      query_contact_configuration_server2_,
      query_contact_configuration_server3_,
      query_contact_configuration_server4_,
      query_contact_configuration_server5_,
      query_contact_configuration_server6_,
      query_contact_configuration_server7_,
      query_contact_configuration_server8_,
      query_contact_configuration_server9_,
      query_contact_configuration_server10_,
      query_contact_configuration_server11_,
      query_contact_configuration_server12_,
      query_contact_configuration_server13_,
      query_contact_configuration_server14_,
      query_contact_configuration_server15_,
      query_contact_configuration_server16_,
      query_contact_configuration_server17_;

  bool goal_dir_flag_ = false;
  bool goal_pos_flag_ = false;
  bool global_path_flag_ = false;
  nav_msgs::Path global_path_;
  Eigen::Quaternionf goal_dir_;
  Eigen::Vector3f goal_pos_;

  SampleSet initial_track_samples_;
  bool track_samples_initialized_ = false;

  float FLIPPER_UPPER_ANGLE = 30.0 / 180.0 * M_PI;   // 30.0 / 180.0 * M_PI
  float FLIPPER_LOWER_ANGLE = -30.0 / 180.0 * M_PI;  // -30.0 / 180.0 * M_PI
  float FLIPPER_RESET_ANGLE = M_PI_4;                // M_PI / 4

  float FLAT_PITCH_THRESHOLD = 12;  // 12
  float DELTA_PITCH_THRESHOLD = 10;
  mutable std::mutex convex_hull_publisher_mutex_, color_pcd_publisher_mutex_,
      vector_pose_publisher_mutex_, track_boundary_publisher_mutex_,
      flipper_boundary_publisher_mutex_;
  mutable std::mutex local_octomap_mutex_, global_octomap_mutex_;
  mutable std::mutex service0_mutex_, service1_mutex_, service2_mutex_,
      service3_mutex_, service4_mutex_, service5_mutex_, service6_mutex_,
      service7_mutex_, service8_mutex_, service9_mutex_;
  bool debug_ = true;
  bool costmap_enable_ = false;
  float lookahead_distance_ = 2.0;
  float track_width_ = 0.075;
  float track_length_ = 0.672;
  float track_separation_ = 0.329;
  float flipper_width_ = 0.040;
  float flipper_length_ = 0.192;
  float flipper_separation_ = 0.45;
  bool global_mode_ = false;
  bool has_costmap_ = false;
  bool strict_contact_ = true;
  Eigen::Isometry3d current_pose_;  // 每得到一次局部地图,在更新高程图前获取tf
  nav_msgs::OccupancyGrid costmap_;  // <100 均可通行
  nav_msgs::OccupancyGrid elevation_map_;
  nav_msgs::OccupancyGrid costmap_dn_;
  nav_msgs::OccupancyGrid elevation_map_dn_;
};
