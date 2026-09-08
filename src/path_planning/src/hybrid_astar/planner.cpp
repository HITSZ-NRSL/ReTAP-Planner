#include "hybrid_astar/planner.h"

using namespace HybridAStar;

Planner::Planner() {
  local_planner_ptr.reset(new LocalPlanning());
  configurationSpace.setLocalPlanner(local_planner_ptr);

  ros::NodeHandle n;
  // n.getParam("hybrid_astar/map_frame", map_frame_);
  // n.getParam("hybrid_astar/base_footprint_frame", base_footprint_frame_);

  subCostMap =
      n.subscribe("local_planning/local_costmap", 1, &Planner::setMap, this);
  subElevMap =
      n.subscribe("local_planning/local_elevation_map", 1, &Planner::setElevationMap, this);

  subGoal = n.subscribe("/move_base_simple/goal", 1,
                        &Planner::clickedNavGoalCallback, this);
  subGlobalPath = n.subscribe("global_planning/path", 1,
                              &Planner::GlobalPathCallback, this);
  subStopCmd = n.subscribe("/stop_cmd", 1, &Planner::stopCommandCB, this);

  pubPath = n.advertise<nav_msgs::Path>("/local_planning/path", 1);
  pubPathPoints =
      n.advertise<sensor_msgs::PointCloud2>("/local_planning/path_points", 1);
  pubConfigPath =
      n.advertise<geometry_msgs::PoseArray>("/local_planning/path_configs", 1);
}

void Planner::GlobalPathCallback(const nav_msgs::PathConstPtr msg) {
  if (!validElevationMap) {
    ROS_ERROR("<GlobalPathCallback>: No map avaliable!!!");
    return;
  }
  nav_msgs::Path path = *msg;
  if (path.poses.size() <= 0) {
    ROS_ERROR("<GlobalPathCallback>: Invalid path !!!");
    return;
  }

  float max_index = 0;
  float acc_dis = 0.8;
  // 在全局路线上选一个最远的无碰撞点
  for (int i = 1; i < path.poses.size(); i++) {
    geometry_msgs::Point p0 = path.poses[i - 1].pose.position;
    geometry_msgs::Point p1 = path.poses[i].pose.position;
    Eigen::Vector3f dis(p1.x - p0.x, p1.y - p0.y, 0);
    auto q = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(), dis);
    path.poses[i].pose.orientation.w = q.w();
    path.poses[i].pose.orientation.x = q.x();
    path.poses[i].pose.orientation.y = q.y();
    path.poses[i].pose.orientation.z = q.z();

    acc_dis -= dis.norm();
    printf("[%d/%d]-th: dis=%.3f. ", i, (int)path.poses.size(), acc_dis);
    bool feasible = setGoal(path.poses[i].pose);
    if (!feasible || acc_dis <= 0) break;
    max_index = i;
  }

  // sleep(3);
  if (max_index <= 0) {
    ROS_ERROR("<GlobalPathCallback>: No feasible goal on the path !!!");
    return;
  }
}

void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }
}

geometry_msgs::Pose eigenIsometryToGeoMsgPose(Eigen::Isometry3d base_pose) {
  geometry_msgs::Pose pose;
  Eigen::Quaterniond q(base_pose.rotation());
  pose.orientation.w = q.w();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.x = q.x();
  pose.position.x = base_pose.translation().x();
  pose.position.y = base_pose.translation().y();
  pose.position.z = base_pose.translation().z();
  return pose;
}

void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  grid = map;
  configurationSpace.updateGrid(grid);
  if (!validElevationMap) return;  // wait for elavation map

  // 接收一次地图，获取一次tf，触发一次规划
  Eigen::Isometry3d base_pose = local_planner_ptr->current_pose_;
  // local_planner_ptr->getSensorPoseEigen(&base_pose, ros::Time(0));
  // if (getSensorPoseEigen(&base_pose, ros::Time::now()))
  {
    auto pose = eigenIsometryToGeoMsgPose(base_pose);
    setStart(pose);
  }
}

void Planner::setElevationMap(const nav_msgs::OccupancyGrid::Ptr map) {
  map->info.origin.position.x += map->info.width * map->info.resolution / 2;
  map->info.origin.position.y += map->info.height * map->info.resolution / 2;
  elevation_map = map;
  configurationSpace.updateElevationMap(elevation_map);
  validElevationMap = true;
}

void Planner::mapPairCallback(
    const nav_msgs::OccupancyGrid::ConstPtr& cost_map,
    const nav_msgs::OccupancyGrid::ConstPtr& elev_map) {
  setMap(boost::make_shared<nav_msgs::OccupancyGrid>(*cost_map));
  setElevationMap(boost::make_shared<nav_msgs::OccupancyGrid>(*elev_map));
}

// isTraversable2D 从高程图查询z，并更新到node3D的z
bool Planner::transToLocalCoordChecked(float x, float y, float z,
                                       geometry_msgs::Quaternion q, float& lx,
                                       float& ly, float& lz, float& yaw) {
  lx = (x - grid->info.origin.position.x) / Constants::cellSize;
  ly = (y - grid->info.origin.position.y) / Constants::cellSize;
  lz = (z - grid->info.origin.position.z) / Constants::cellSize;  // add by liyx
  yaw = Helper::normalizeHeadingRad(
      tf::getYaw(q));  // set theta to a value (0,2PI]

  auto node = Node3D(lx, ly, lz, yaw, 0, 0, nullptr);
  if (configurationSpace.isTraversable2D(&node)) {
    lz = node.getZ();
    return true;
  }
  return false;
}

bool Planner::setStart(geometry_msgs::Pose pose) {
  float lx, ly, lz, yaw;
  bool feasible = transToLocalCoordChecked(pose.position.x, pose.position.y,
                                           pose.position.z, pose.orientation,
                                           lx, ly, lz, yaw);
  // lz = 0;  // TODO(liyx): base_link改为base_footprint，这里去掉
  if (feasible) {
    validStart = true;
    nStart = Node3D(lx, ly, lz, yaw, 0, 0, nullptr);
    printf("<Planner::plan>: Start(%.3f,%.3f,%.3f,%.3f)\n", lx, ly, lz, yaw);
    return true;
  }
  ROS_ERROR("<Planner::plan>: Invalid Start(%.3f,%.3f,%.3f,%.3f)\n", lx, ly, lz,
            yaw);
  return false;
}

bool Planner::setGoal(geometry_msgs::Pose pose) {
  float lx, ly, lz, yaw;
  bool feasible = transToLocalCoordChecked(pose.position.x, pose.position.y,
                                           pose.position.z, pose.orientation,
                                           lx, ly, lz, yaw);
  if (feasible) {
    validGoal = true;
    nGoal = Node3D(lx, ly, lz, yaw, 0, 0, nullptr);
    printf("<Planner::plan>: Checked Goal(%.3f,%.3f,%.3f,%.3f)\n", lx, ly, lz,
           yaw);
    return true;
  }

  printf("<Planner::plan>: Invalid Goal(%.3f,%.3f,%.3f,%.3f)\n", lx, ly, lz,
         yaw);
  return false;
}

void Planner::clickedNavGoalCallback(
    const geometry_msgs::PoseStamped::ConstPtr& end) {
  if (!validElevationMap) {
    ROS_ERROR("<clickedNavGoalCallback>: No map avaliable!!!");
    return;
  }
  if (setGoal(end->pose) && Constants::manual) {
    plan();
  }
  if (0) {
    float x = (end->pose.position.x - grid->info.origin.position.x) /
              Constants::cellSize;
    float y = (end->pose.position.y - grid->info.origin.position.y) /
              Constants::cellSize;
    float z = (end->pose.position.z - grid->info.origin.position.z) /
              Constants::cellSize;  // add by liyx
    float t = Helper::normalizeHeadingRad(
        tf::getYaw(end->pose.orientation));  // set theta to a value (0,2PI]

    std::cout << "I am seeing a new goal x:" << x << " y:" << y
              << " t:" << Helper::toDeg(t) << std::endl;

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      validGoal = true;
      nGoal = Node3D(x, y, z, t, 0, 0, nullptr);
      printf("<Planner::plan>: iGoal(%.3f,%.3f,%.3f,%.3f), ", x, y, z, t);

      if (Constants::manual) {
        plan();
      }
    } else {
      std::cout << "invalid goal x:" << x << " y:" << y
                << " t:" << Helper::toDeg(t) << std::endl;
    }
  }
}

void Planner::tracePathNodes(const Node3D* node, int i,
                             std::vector<Node3D> path_nodes) {
  if (node == nullptr) {
    this->path_nodes = path_nodes;
    return;
  }

  if (node->getMid().flag) {
    printf("[%02d]: mid: x=%.3f, y=%.3f, z=%.3f, t=%.3f | f=%.3f, l=%.3f\n", i,
           node->getMid().x, node->getMid().y, node->getMid().z,
           node->getMid().t, node->getMid().ff, node->getMid().rf);
  }

  printf("[%02d]: seq: x=%.3f, y=%.3f, z=%.3f, t=%.3f | f=%.3f, l=%.3f\n", i,
         node->getX(), node->getY(), node->getZ(), node->getT(),
         node->getFlipperAngles().first, node->getFlipperAngles().second);
  i++;
  path_nodes.push_back(*node);
  tracePathNodes(node->getPred(), i, path_nodes);
}

void Planner::addPathPoint(float x, float y, float z, float t, float ff,
                           float rf, Node3D* node) {
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  // pose.pose.orientation = tf::createQuaternionMsgFromYaw(t);
  pose.pose.orientation.x = node->getPose().x();
  pose.pose.orientation.y = node->getPose().y();
  pose.pose.orientation.z = node->getPose().z();
  pose.pose.orientation.w = node->getPose().w();
  path.poses.push_back(pose);

  geometry_msgs::Pose config = pose.pose;  // add by liyx
  config.orientation.x = ff;
  config.orientation.y = rf;
  config.orientation.z = t;
  config_path.poses.push_back(config);

  pcl::PointXYZI p;
  p.x = pose.pose.position.x;
  p.y = pose.pose.position.y;
  p.z = pose.pose.position.z;
  // p.intensity = i;
  path_points.push_back(p);
}

void Planner::publishPath(geometry_msgs::Point offset) {
  path.poses.clear();
  config_path.poses.clear();
  path_points.clear();
  path.header.stamp = ros::Time::now();
  path.header.frame_id = local_planner_ptr->map_frame_;
  config_path.header = path.header;
  for (int i = path_nodes.size() - 1; i >= 0; i--) {
    auto node = path_nodes[i];

    if (node.getMid().flag) {
      float x = node.getMid().x * Constants::cellSize + offset.x;
      float y = node.getMid().y * Constants::cellSize + offset.y;
      float z = node.getMid().z * Constants::cellSize + offset.z;
      addPathPoint(x, y, z, node.getMid().t, node.getMid().ff, node.getMid().rf,
                   &node);
    }

    float x = node.getX() * Constants::cellSize + offset.x;
    float y = node.getY() * Constants::cellSize + offset.y;
    float z = node.getZ() * Constants::cellSize + offset.z;
    float t = node.getT();
    float ff = node.getFlipperAngles().first;
    float rf = node.getFlipperAngles().second;
    addPathPoint(x, y, z, t, ff, rf, &node);
  }

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(path_points, msg);
  msg.header.frame_id = local_planner_ptr->map_frame_;
  msg.header.stamp = ros::Time::now();
  pubPathPoints.publish(msg);
  pubPath.publish(path);
  pubConfigPath.publish(config_path);

  std::cout << "<publishPath>: " << std::endl;
  for (int i = 0; i < config_path.poses.size(); i++) {
    printf("[%d]:(x,y,z,f,r)=(%.2f,%.2f,%.2f,%.2f,%.2f)\n", i,
           config_path.poses[i].position.x, config_path.poses[i].position.y,
           config_path.poses[i].position.z, config_path.poses[i].orientation.x,
           config_path.poses[i].orientation.y);
  }
}

bool Planner::plan() {
  // validStart从costmap和elevation map话题的callback中的tf订阅使能
  // validGoal从2D nav goal或global_path使能
  if (local_planner_ptr->has_costmap_) {  // check map from local_planner
    local_planner_ptr->has_costmap_ = false;
    nav_msgs::OccupancyGrid::Ptr costmap_ptr, elevation_map_ptr;
    costmap_ptr.reset(new nav_msgs::OccupancyGrid);
    elevation_map_ptr.reset(new nav_msgs::OccupancyGrid);
    *costmap_ptr = local_planner_ptr->costmap_dn_;              // clone
    *elevation_map_ptr = local_planner_ptr->elevation_map_dn_;  // clone
    setMap(costmap_ptr);
    setElevationMap(elevation_map_ptr);
  }
  if (validStart && validGoal) {
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;  // 方向个数
    int length = width * height * depth;
    printf(
        "<Planner::plan>: Grid size: width=%d, height=%d, "
        "headings=%d ==> %d nodes\n",
        width, height, depth, length);

    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    ros::WallTime t0 = ros::WallTime::now();  // START AND TIME THE PLANNING

    // FIND THE PATH
    Node3D* nSolution =
        Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height,
                               configurationSpace, dubinsLookup);

    if (nSolution == nullptr) {
      ROS_ERROR("nSolution is nullptr. No result found.");
    } else {
      printf("<tracePathNodes>: \n");
      tracePathNodes(nSolution);
      publishPath(grid->info.origin.position);
    }

    ros::WallDuration d = ros::WallTime::now() - t0;
    printf("<Planner::plan>: Timecost: %.4fs, Path size: %d\n", d.toSec(),
           static_cast<int>(path.poses.size()));

    delete[] nodes3D;
    delete[] nodes2D;
    validGoal = false;
    validStart = false;
    validElevationMap = false;
    if (path.poses.size() <= 0) {
      ROS_ERROR("No path found.");
      return false;
    }
    return true;
  } else {
    // std::cout << "missing goal or start" << std::endl;
  }
  return false;
}

void Planner::stopCommandCB(const std_msgs::Int8ConstPtr msg) {
  validGoal = false;
  validStart = false;
  std::cout << "<stopCommandCB>: stop planning." << std::endl;
}
