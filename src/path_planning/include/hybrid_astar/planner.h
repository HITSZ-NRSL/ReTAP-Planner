#ifndef PLANNER_H
#define PLANNER_H
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ctime>
#include <iostream>

#include "hybrid_astar/algorithm.h"
#include "hybrid_astar/constants.h"
#include "hybrid_astar/helper.h"
#include "hybrid_astar/lookup.h"
#include "hybrid_astar/node3d.h"

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.

    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be
   used with the ROS navigation stack \todo make it actually inherit from
   nav_core::BaseGlobalPlanner
*/
class Planner {
 public:
  /// The default constructor
  Planner();
  ~Planner() { delete[] dubinsLookup; }

  // Initializes the collision as well as heuristic lookup table probably
  // removed
  void initializeLookups();

  void setMap(const nav_msgs::OccupancyGrid::Ptr map);
  void setElevationMap(const nav_msgs::OccupancyGrid::Ptr map);
  void mapPairCallback(const nav_msgs::OccupancyGrid::ConstPtr& cost_map,
                       const nav_msgs::OccupancyGrid::ConstPtr& elev_map);
  void clickedNavGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
  bool setGoal(geometry_msgs::Pose pose);
  bool setStart(geometry_msgs::Pose pose);
  void stopCommandCB(const std_msgs::Int8ConstPtr msg);

  bool plan();
  bool planForGlobalPath();
  void clientTest();

  bool getSensorPoseTF(tf::StampedTransform* pose, ros::Time timestamp);
  bool getSensorPoseEigen(Eigen::Isometry3d* pose, ros::Time timestamp);
  void odomTimerCallback(const ros::TimerEvent& event);
  void GlobalPathCallback(const nav_msgs::PathConstPtr msg);
  void setFrame(std::string frame) { map_frame_ = frame; }
  void tracePathNodes(const Node3D* node, int i = 0,
                      std::vector<Node3D> path_nodes = std::vector<Node3D>());
  void publishPath(geometry_msgs::Point offset);
  void addPathPoint(float x, float y, float z, float t, float ff, float rf,
                    Node3D* node);
  bool transToLocalCoordChecked(float x, float y, float z,
                                geometry_msgs::Quaternion q, float& lx,
                                float& ly, float& lz, float& yaw);

 private:
  std::shared_ptr<LocalPlanning> local_planner_ptr;
  ros::Subscriber subCostMap, subElevMap;
  message_filters::Subscriber<nav_msgs::OccupancyGrid>* subCostMapPtr;
  message_filters::Subscriber<nav_msgs::OccupancyGrid>* subElevMapPtr;
  typedef message_filters::sync_policies::ApproximateTime<
      nav_msgs::OccupancyGrid, nav_msgs::OccupancyGrid>
      occupancy_sync_policy;
  message_filters::Synchronizer<occupancy_sync_policy>* syncPtr;
  ros::Subscriber subStart, subGoal, subGlobalPath, subStopCmd;
  ros::Publisher pubPath, pubConfigPath, pubPathPoints;
  tf::TransformListener listener;
  tf::StampedTransform transform;

  /// A pointer to the grid the planner runs on
  nav_msgs::OccupancyGrid::Ptr grid, elevation_map;
  /// The start pose set through RViz
  //   geometry_msgs::PoseWithCovarianceStamped start;
  /// The goal pose set through RViz
  //   geometry_msgs::PoseStamped goal;
  /// Flags for allowing the planner to plan
  bool validStart = false;
  /// Flags for allowing the planner to plan
  bool validGoal = false;

  /// The collission detection for testing specific configurations
  CollisionDetection configurationSpace;
  /// A lookup of analytical solutions (Dubin's paths)
  float* dubinsLookup =
      new float[Constants::headings * Constants::headings *
                Constants::dubinsWidth * Constants::dubinsWidth];

  std::string map_frame_ = "map";
  std::string base_footprint_frame_ = "base_link";
  bool validElevationMap = false;

  /// The path produced by the hybrid A* algorithm
  std::vector<Node3D> path_nodes;
  nav_msgs::Path path;  // Path data structure for visualization
  pcl::PointCloud<pcl::PointXYZI> path_points;
  geometry_msgs::PoseArray
      config_path;  // configuration sequence with flipper angles
  Node3D nGoal, nStart;
};
}  // namespace HybridAStar
#endif  // PLANNER_H
