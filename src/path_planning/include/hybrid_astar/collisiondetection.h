#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <omp.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "hybrid_astar/constants.h"
#include "hybrid_astar/lookup.h"
#include "hybrid_astar/node2d.h"
#include "hybrid_astar/node3d.h"
#include "local_planning/local_planning.h"

namespace HybridAStar {
namespace {
inline void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  // avoid 2D collision checking
  t = 99;
}

// 从node中取出X、Y、T
inline void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = node->getT();
}
}  // namespace
/*!
   \brief The CollisionDetection class determines whether a given configuration
   q of the robot will result in a collision with the environment.

   It is supposed to return a boolean value that returns true for collisions and
   false in the case of a safe node.
*/
class CollisionDetection {
 public:
  /// Constructor
  CollisionDetection() {
    Lookup::collisionLookup(collisionLookup);

    // ros::NodeHandle n;
    // for (int i = 0; i < 9; i++) {
    //   std::stringstream service_name;
    //   service_name << "local_planning/query_contact_configuration" << i;
    //   clients[i] =
    //   n.serviceClient<path_planning::query_contact_configuration>(
    //       service_name.str());
    // }
  }

  void setLocalPlanner(std::shared_ptr<LocalPlanning> planner_ptr) {
    local_planner_ptr = planner_ptr;
  }

  /*!
     \brief evaluates whether the configuration is safe
     \return true if it is traversable, else false
  */
  bool isTraversable(const Node2D* node) {
    /* Depending on the used collision checking mechanism this needs to be
       adjusted standard: collision checking using the spatial occupancy
       enumeration other: collision checking using the 2d costmap and the
       navigation stack
    */
    float x, y, t;
    // assign values to the configuration
    getConfiguration(node, x, y, t);

    // 2D collision test
    // return grid->data[node->getIdx()] == 0;
    return grid->data[node->getIdx()] < 100;
  }

  bool isTraversable2D(Node3D* node) {
    float x, y, z, t;
    getConfiguration(node, x, y, t);  // assign values to the configuration

    if (x < 0 || (unsigned int)x >= grid->info.width  //
        || y < 0 || (unsigned int)y >= grid->info.height) {
      return false;
    }

    node->setIdx(grid->info.width, grid->info.height);
    if (grid->data[node->getIdx2D()] == 100) return false;
    // 从elevation_map中获取z
    z = elevation->data[node->getIdx2D()];
    if (z == -128) {          // 高度未知时
      if (node->getPred()) {  // 用父节点的高度
        node->setZ(node->getPred()->getZ());
      } else {  // 没有父节点返回false
        return false;
      }
    } else {
      node->setZ(z);
    }
    return true;
  }

  /* Depending on the used collision checking mechanism this needs to be
      adjusted standard: collision checking using the spatial occupancy
      enumeration other: collision checking using the 2d costmap and the
      navigation stack
  */
  bool isTraversable(Node3D* node) {
    float x, y, z, t;
    getConfiguration(node, x, y, t);  // assign values to the configuration

    if (x < 0 || (unsigned int)x >= grid->info.width  //
        || y < 0 || (unsigned int)y >= grid->info.height) {
      return false;
    }

    node->setIdx(grid->info.width, grid->info.height);
    // if (grid->data[node->getIdx2D()] != 0) return false;
    if (grid->data[node->getIdx2D()] == 100) return false;

    // 从elevation_map中获取z
    z = elevation->data[node->getIdx2D()];
    if (z == -128) {          // 高度未知时
      if (node->getPred()) {  // 用父节点的高度
        node->setZ(node->getPred()->getZ());
      } else {  // 没有父节点返回false
        return false;
      }
    } else {
      node->setZ(z);
    }
    if (1) {
      geometry_msgs::Point p;
      p.x = x * grid->info.resolution + grid->info.origin.position.x;
      p.y = y * grid->info.resolution + grid->info.origin.position.y;
      p.z = z * grid->info.resolution + grid->info.origin.position.z;

      geometry_msgs::Quaternion q =
          tf::createQuaternionMsgFromYaw(t);  // 将yaw转为四元数

      bool debug = false;
      if (debug) {
        printf(
            "<isTraversable(3D)>: (%.2f,%.2f,%.2f), yaw:%.2f, "
            "q:(%.2f,%.2f,%.2f,%.2f)\n",
            p.x, p.y, p.z, t, q.w, q.x, q.y, q.z);
      }

      ConfigurationPoint way_point;
      way_point.position = Eigen::Vector3f(p.x, p.y, p.z);
      way_point.orientation = Eigen::Quaternionf(q.w, q.x, q.y, q.z);

      ros::WallTime t = ros::WallTime::now();
      if (!local_planner_ptr->PlanConfiguration(way_point)) {
        ROS_ERROR("no feasible configuration");
        return false;
      }

      node->setFlipperAngles(way_point.flipper_angles.first,
                             way_point.flipper_angles.second);
      // node->setOffsetZ(way_point.offset.z());
      node->setZ(node->getZ() + way_point.offset.z() / Constants::cellSize);
      node->setPose(way_point.pose);

      if (debug) printf("get feasible configuration\n");
    }
    return true;
  }

  /*!
     \brief updates the grid with the world map
  */
  void updateGrid(nav_msgs::OccupancyGrid::Ptr map) { grid = map; }
  void updateElevationMap(nav_msgs::OccupancyGrid::Ptr map) { elevation = map; }

 private:
  /// The occupancy grid
  nav_msgs::OccupancyGrid::Ptr grid = nullptr;
  nav_msgs::OccupancyGrid::Ptr elevation = nullptr;
  /// The collision lookup table
  Constants::config
      collisionLookup[Constants::headings *
                      Constants::positions];  // headings * resolutions *
                                              // resolutions
  ros::ServiceClient clients[9];
  std::shared_ptr<LocalPlanning> local_planner_ptr = nullptr;
  bool use_service = false;
};
}  // namespace HybridAStar
#endif  // COLLISIONDETECTION_H
