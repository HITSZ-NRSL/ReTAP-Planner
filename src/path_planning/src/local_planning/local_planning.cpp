/* Copyright Year: 2026
 * Copyright Owner: Networked Robotics and Systems Lab
 * Authors: Yuxiang Li, Kun Chen, Haoyao Chen
 */
 
#include "local_planning/local_planning.h"

#include "local_planning/pcd_utils.h"
#include "local_planning/quick_hull.h"

LocalPlanning::LocalPlanning() {
  ros::NodeHandle nh_;
  nh_.getParam("local_planning_node/map_frame", map_frame_);
  nh_.getParam("local_planning_node/base_footprint_frame",
               base_footprint_frame_);
  nh_.getParam("local_planning_node/lookahead_distance", lookahead_distance_);
  nh_.getParam("local_planning_node/debug", debug_);
  nh_.getParam("local_planning_node/track_width", track_width_);
  nh_.getParam("local_planning_node/track_length", track_length_);
  nh_.getParam("local_planning_node/track_separation", track_separation_);
  nh_.getParam("local_planning_node/flipper_width", flipper_width_);
  nh_.getParam("local_planning_node/flipper_length", flipper_length_);
  nh_.getParam("local_planning_node/flipper_separation", flipper_separation_);
  nh_.getParam("local_planning_node/costmap", costmap_enable_);
  nh_.getParam("local_planning_node/global_mode", global_mode_);

  track_rotation_axis_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "local_planning/track_rotation_axis", 1);
  if (global_mode_) {
    path_publisher_ = nh_.advertise<nav_msgs::Path>("local_planning/path", 1);
    pose_array_publisher_ = nh_.advertise<geometry_msgs::PoseArray>(
        "local_planning/pose_array_output", 1);

    if (1) {
      global_path_subscriber_ = nh_.subscribe(
          "global_planning/path", 1, &LocalPlanning::GlobalPathCallback, this);
    } else {
      local_pose_array_subscrber_ =
          nh_.subscribe("local_planning/pose_array_input", 1,
                        &LocalPlanning::LocalPoseArrayCallback, this);
    }
  }
  if (debug_) {
    goal_dir_subscriber_ = nh_.subscribe("/move_base_simple/goal", 1,
                                         &LocalPlanning::GoalDirCallback, this);
    goal_pos_subscriber_ = nh_.subscribe("/clicked_point", 1,
                                         &LocalPlanning::GoalPosCallback, this);
    goal_3d_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "local_planning/nav_goal_3d", 1);
    goal_norm_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "local_planning/nav_goal_normal", 1);
  }

  if (costmap_enable_) {
    local_costmap_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>(
        "local_planning/local_costmap", 1);
    local_elevation_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>(
        "local_planning/local_elevation_map", 1);
  }
  // odom_subscriber_ =
  //     nh_.subscribe("/odometry_gt_10hz", 1, &LocalPlanning::OdomCallback,
  //     this);
  local_octomap_subscriber_ =
      nh_.subscribe("mapping_module/local_octree", 1,
                    &LocalPlanning::LocalOctomapCallback, this);
  global_octomap_subscriber_ =
      nh_.subscribe("mapping_module/global_octree", 1,
                    &LocalPlanning::GlobalOctomapCallback, this);

  track_boundry_publisher = nh_.advertise<visualization_msgs::Marker>(
      "local_planning/track_boundry_markers", 100);
  flipper_boundry_publisher = nh_.advertise<visualization_msgs::Marker>(
      "local_planning/flipper_boundry_markers", 100);
  track_convex_hull_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      "local_planning/track_convex_hull_markers", 100);
  base_convex_hull_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      "local_planning/base_convex_hull_markers", 100);

  track_contact_pcd_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "local_planning/track_contact_pcd", 1);
  flipper_contact_pcd_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "local_planning/flipper_contact_pcd", 1);
  // flipper_angle_publisher_ =
  //     nh_.advertise<geometry_msgs::Twist>("/cmd_flipper", 1);
  query_contact_configuration_server_ = nh_.advertiseService(
      "local_planning/query_contact_configuration",
      &LocalPlanning::queryContactConfigurationServerCallback, this);
  query_contact_configuration_server0_ = nh_.advertiseService(
      "local_planning/query_contact_configuration0",
      &LocalPlanning::queryContactConfigurationServer0Callback, this);
  query_contact_configuration_server1_ = nh_.advertiseService(
      "local_planning/query_contact_configuration1",
      &LocalPlanning::queryContactConfigurationServer1Callback, this);
  query_contact_configuration_server2_ = nh_.advertiseService(
      "local_planning/query_contact_configuration2",
      &LocalPlanning::queryContactConfigurationServer2Callback, this);
  query_contact_configuration_server3_ = nh_.advertiseService(
      "local_planning/query_contact_configuration3",
      &LocalPlanning::queryContactConfigurationServer3Callback, this);
  query_contact_configuration_server4_ = nh_.advertiseService(
      "local_planning/query_contact_configuration4",
      &LocalPlanning::queryContactConfigurationServer4Callback, this);
  query_contact_configuration_server5_ = nh_.advertiseService(
      "local_planning/query_contact_configuration5",
      &LocalPlanning::queryContactConfigurationServer5Callback, this);
  query_contact_configuration_server6_ = nh_.advertiseService(
      "local_planning/query_contact_configuration6",
      &LocalPlanning::queryContactConfigurationServer6Callback, this);
  query_contact_configuration_server7_ = nh_.advertiseService(
      "local_planning/query_contact_configuration7",
      &LocalPlanning::queryContactConfigurationServer7Callback, this);
  query_contact_configuration_server8_ = nh_.advertiseService(
      "local_planning/query_contact_configuration8",
      &LocalPlanning::queryContactConfigurationServer8Callback, this);
  // query_contact_configuration_server9_ = nh_.advertiseService(
  //     "local_planning/query_contact_configuration9",
  //     &LocalPlanning::queryContactConfigurationServerCallback, this);
  // query_contact_configuration_server10_ = nh_.advertiseService(
  //     "local_planning/query_contact_configuration10",
  //     &LocalPlanning::queryContactConfigurationServerCallback, this);
  // query_contact_configuration_server11_ = nh_.advertiseService(
  //     "local_planning/query_contact_configuration11",
  //     &LocalPlanning::queryContactConfigurationServerCallback, this);
  // query_contact_configuration_server12_ = nh_.advertiseService(
  //     "local_planning/query_contact_configuration12",
  //     &LocalPlanning::queryContactConfigurationServerCallback, this);
  // query_contact_configuration_server13_ = nh_.advertiseService(
  //     "local_planning/query_contact_configuration13",
  //     &LocalPlanning::queryContactConfigurationServerCallback, this);
  // query_contact_configuration_server14_ = nh_.advertiseService(
  //     "local_planning/query_contact_configuration14",
  //     &LocalPlanning::queryContactConfigurationServerCallback, this);
  // query_contact_configuration_server15_ = nh_.advertiseService(
  //     "local_planning/query_contact_configuration15",
  //     &LocalPlanning::queryContactConfigurationServerCallback, this);
  // query_contact_configuration_server16_ = nh_.advertiseService(
  //     "local_planning/query_contact_configuration16",
  //     &LocalPlanning::queryContactConfigurationServerCallback, this);
  // query_contact_configuration_server17_ = nh_.advertiseService(
  //     "local_planning/query_contact_configuration17",
  //     &LocalPlanning::queryContactConfigurationServerCallback, this);
  // initial_track_samples_ = SampleSet(0.075,   // _track_width
  //                                    0.672,   // _track_length
  //                                    0.329,   // _track_separation
  //                                    0.040,   // _flipper_width
  //                                    0.192,   // _flipper_length
  //                                    0.450,   // _flipper_separation
  //                                    0.025);  // sample_resolution
}

bool LocalPlanning::getBasePoseTF(tf::StampedTransform* pose,
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

bool LocalPlanning::getBasePoseEigen(Eigen::Isometry3d* pose,
                                     ros::Time timestamp) {
  tf::StampedTransform lidar_transform_tf;
  if (!getBasePoseTF(&lidar_transform_tf, timestamp)) {
    return false;
  }
  tf::transformTFToEigen(lidar_transform_tf, *pose);

  return true;
}

octomap::OcTreeKey LocalPlanning::decentralizeKey(octomap::OcTreeKey key,
                                                  octomap::OcTreeKey center) {
  auto zero_key = local_octree_ptr_->coordToKey(0, 0, 0);
  int dk_2 = key[2] - center[2];
  int dk_1 = key[1] - center[1];
  int dk_0 = key[0] - center[0];
  key[2] = dk_2 + zero_key[2];
  key[1] = dk_1 + zero_key[1];
  key[0] = dk_0 + zero_key[0];
  return key;
}

octomap::OcTreeKey LocalPlanning::translateKey(octomap::OcTreeKey key,
                                               octomap::OcTreeKey center) {
  auto zero_key = local_octree_ptr_->coordToKey(0, 0, 0);
  int dk_2 = key[2] - zero_key[2];
  int dk_1 = key[1] - zero_key[1];
  int dk_0 = key[0] - zero_key[0];
  key[2] = dk_2 + center[2];
  key[1] = dk_1 + center[1];
  key[0] = dk_0 + center[0];
  return key;
}

void resizeGridMap(nav_msgs::OccupancyGrid& costmap_in,
                   nav_msgs::OccupancyGrid& costmap_out,
                   nav_msgs::OccupancyGrid& elevation_map_in,
                   nav_msgs::OccupancyGrid& elevation_map_out) {
  int coef = 4;
  float res = costmap_in.info.resolution * coef;
  int width = round(costmap_in.info.width / coef);
  int height = round(costmap_in.info.height / coef);
  int size = width * height;

  elevation_map_out = elevation_map_in;
  elevation_map_out.info.resolution = res;
  elevation_map_out.info.width = width;
  elevation_map_out.info.height = height;
  elevation_map_out.data.clear();
  elevation_map_out.data.resize(size, -2);  // map: yellow
  // costmap_out = elevation_map_out;
  for (int i = 0; i < size; i++) {
    elevation_map_out.data[i] = -128;  // costmap: red
  }
  costmap_out = elevation_map_out;

  for (int ix = 0; ix < costmap_in.info.width; ix++) {
    for (int iy = 0; iy < costmap_in.info.height; iy++) {
      int id = iy * costmap_in.info.width + ix;
      int ox = ix / coef;
      int oy = iy / coef;
      int od = oy * width + ox;
      costmap_out.data[od] =
          std::max(costmap_out.data[od], costmap_in.data[id]);
      if (elevation_map_in.data[id] != -128) {
        elevation_map_out.data[od] =
            std::max(elevation_map_out.data[od] * 1.0,
                     (elevation_map_in.data[id] * 1.0 / coef));
      }
    }
  }
}

void LocalPlanning::LocalOctomapToElevationGrid() {
  const int image_size = 160;
  const int offset = 80;
  const float resolution = 0.025;

  while (!getBasePoseEigen(&current_pose_, ros::Time(0))) {
    sleep(1);
  }

  if (!global_octree_ptr_ || !global_octree_ptr_->getRoot() ||
      !global_octree_ptr_->getRoot()->hasChildren()) {
    ROS_ERROR("<LocalOctomapToElevationGrid>: global_octree is not avaliable.");
    return;
  }

  auto center_key = local_octree_ptr_->coordToKey(
      current_pose_.translation().x(), current_pose_.translation().y(),
      current_pose_.translation().z());
  octomap::point3d center = local_octree_ptr_->keyToCoord(center_key);

  float bbox_size = 4;
  float bbox_height = 2;
  octomap::Boundingbox bbox;
  bbox.insertPoint(
      octomap::point3d(current_pose_.translation().x() + bbox_size * 0.5,
                       current_pose_.translation().y() + bbox_size * 0.5,
                       current_pose_.translation().z() + bbox_height * 0.5));
  bbox.insertPoint(
      octomap::point3d(current_pose_.translation().x() - bbox_size * 0.5,
                       current_pose_.translation().y() - bbox_size * 0.5,
                       current_pose_.translation().z() - bbox_height * 0.5));
  bbox.enlarge(0.5);

  std::unordered_map<octomap::OcTreeKey, octomap::key_type,
                     octomap::OcTreeKey::KeyHash>
      elevation_map;
  auto start =
      local_octree_ptr_->begin_leafs_bbx(bbox.minPoint(), bbox.maxPoint());
  auto end = local_octree_ptr_->end_leafs_bbx();
  // 对栅格去中心，并去重
  for (auto iter = start; iter != end; iter++) {
    auto node = local_octree_ptr_->search(iter.getKey());
    if (!node || !local_octree_ptr_->isNodeOccupied(node)) {
      continue;
    }
    auto key = decentralizeKey(iter.getKey(), center_key);
    auto p = local_octree_ptr_->keyToCoord(key);

    // 相对高度, 如果超过阈值则忽略, ±1.5
    if (fabs(p.z()) > bbox_height * 0.5) {  //
      continue;  // Ignore points that are too high or too low
    }
    // 对高程点云去重，避免高程图的重复高度、重复评估构型
    auto kz = key.k[2];
    key.k[2] = 0;
    if (elevation_map.count(key) == 0 || kz > elevation_map[key]) {
      elevation_map[key] = kz;
    }
  }

  if (elevation_map.size() <= 0) {
    ROS_ERROR("<LocalOctomapToElevationGrid>: No points in the local bbox.");
    has_costmap_ = false;
    return;
  }

  // 转为将unordered_map转为pcd
  pcl::PointCloud<pcl::PointXYZ> pcd;
  for (auto element : elevation_map) {
    octomap::OcTreeKey key3d = element.first;
    key3d.k[2] = element.second;
    auto p = local_octree_ptr_->keyToCoord(key3d);
    pcl::PointXYZ pc(p.x(), p.y(), p.z());
    pcd.push_back(pc);
  }
  std::cout << "<OctomapToElevationGrid>: elevation pcd size=" << pcd.size()
            << " / " << image_size * image_size << std::endl;
  // 构建costmap地图
  float res = local_octree_ptr_->getResolution();

  int width = bbox_size / res;
  int height = bbox_size / res;
  costmap_.info.origin.position.x = current_pose_.translation().x();  // tf
  costmap_.info.origin.position.y = current_pose_.translation().y();
  costmap_.info.origin.position.z = current_pose_.translation().z();
  costmap_.info.origin.position.x -= bbox_size / 2;
  costmap_.info.origin.position.y -= bbox_size / 2;
  costmap_.info.width = width;
  costmap_.info.height = height;
  costmap_.info.map_load_time = ros::Time::now();
  costmap_.info.resolution = res;
  costmap_.data.clear();
  costmap_.data.resize(height * width, -2);  // map: yellow
  costmap_.header.stamp = ros::Time::now();
  costmap_.header.frame_id = map_frame_;
  elevation_map_ = costmap_;
  // elevation_map_.data.resize(height * width, -128);  // costmap: red
  for (int i = 0; i < height * width; i++) {
    elevation_map_.data[i] = -128;
  }

  std::vector<geometry_msgs::Point> query_pcd;
  std::vector<int> idx_vec;
  int offset_x = width / 2;
  int offset_y = height / 2;
  std::unordered_set<int> idx_set;

  for (auto pc : pcd.points) {
    auto key = local_octree_ptr_->coordToKey(pc.x, pc.y, pc.z);
    auto zkey = local_octree_ptr_->coordToKey(0, 0, 0);
    unsigned int ix = offset + key[0] - zkey[0];
    unsigned int iy = offset + key[1] - zkey[1];
    int iz = key[2] - zkey[2];
    if (ix < 0 || iy < 0 || ix >= image_size || iy >= image_size || iz >= 127 ||
        iz <= -127)  // 超出范围
      continue;

    int idx = iy * width + ix;  // idx = y * width + x
    if (idx_set.find(idx) != idx_set.end()) {
      // 已经更新过，而且高度比原来低，跳过
      if (iz < elevation_map_.data[idx]) continue;
    } else {
      idx_set.insert(idx);
    }
    elevation_map_.data[idx] = iz;  // costmap: 0 trasparent, >0 blue, <0 yellow
    idx_vec.push_back(idx);

    auto key_ori = translateKey(key, center_key);
    auto pc_ori = local_octree_ptr_->keyToCoord(key_ori);
    geometry_msgs::Point p;
    p.x = pc_ori.x();
    p.y = pc_ori.y();
    p.z = pc_ori.z();
    query_pcd.push_back(p);
  }

  std::lock_guard<std::mutex> lock(global_octomap_mutex_);
  elevation_map.clear();  // 同样需要2D去重，取最高点的可通行度
  // 否则高台阶会出现不可通行的条形区域（小格子穿越了底层的大格子）
  for (int i = 0; i < query_pcd.size(); i++) {
    auto p = query_pcd[i];
    auto key = global_octree_ptr_->coordToKey(p.x, p.y, p.z);
    auto node = global_octree_ptr_->search(key);
    if (!node || !global_octree_ptr_->isNodeOccupied(node))
      continue;  // 忽略不能对应占据大格子的小格子

    auto kz = key.k[2];
    key.k[2] = 0;
    if (elevation_map.count(key) == 0 || kz > elevation_map[key]) {
      elevation_map[key] = kz;
    }
  }

  // 从全局地图中查，应该在2D上做查询
  for (int i = 0; i < query_pcd.size(); i++) {
    auto p = query_pcd[i];
    auto key = global_octree_ptr_->coordToKey(p.x, p.y, p.z);
    auto node = global_octree_ptr_->search(key);

    if (!node || !global_octree_ptr_->isNodeOccupied(node)) {
      costmap_.data[idx_vec[i]] = -1;
      continue;
    }

    key.k[2] = 0;
    auto kz = elevation_map.find(key);
    if (kz == elevation_map.end()) {
      costmap_.data[idx_vec[i]] = -1;
      continue;
    }

    key.k[2] = kz->second;
    node = global_octree_ptr_->search(key);  // 重新search
    if (!node || !global_octree_ptr_->isNodeOccupied(node)) {
      costmap_.data[idx_vec[i]] = -1;
      continue;
    }

    if (node->getCollision()) {
      costmap_.data[idx_vec[i]] = 100;
      continue;
    }
    costmap_.data[idx_vec[i]] =
        std::min(node->getTraversability() * 100, 127.0f);
  }

  resizeGridMap(costmap_, costmap_dn_, elevation_map_, elevation_map_dn_);
  local_costmap_publisher_.publish(costmap_dn_);
  local_elevation_publisher_.publish(elevation_map_dn_);
  has_costmap_ = true;
  std::cout << "<OctomapToElevationGrid>: set has_costmap_ true" << std::endl;
}

void LocalPlanning::GlobalOctomapCallback(const octomap_msgs::Octomap& msg) {
  octomap::IgTree temp_ig_tree(0.5);
  std::lock_guard<std::mutex> lock(global_octomap_mutex_);
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  if (aot) {
    if (!global_octree_ptr_) global_octree_ptr_.reset(new octomap::IgTree(0.2));
    global_octree_ptr_.reset(dynamic_cast<octomap::IgTree*>(aot));
  }
  // std::cout << "LocalPlanning::GlobalOctomapCallback" << std::endl;
}

void LocalPlanning::InitTrackSamples() {
  if (!track_samples_initialized_) {
    track_samples_initialized_ = true;
    initial_track_samples_ =
        SampleSet(track_width_, track_length_, track_separation_,
                  flipper_width_, flipper_length_, flipper_separation_,
                  local_octree_ptr_->getResolution());  // sample_resolution

    // TODO(liyx): Get Params:
    // FLIPPER_UPPER_ANGLE, FLIPPER_LOWER_ANGLE, FLIPPER_RESET_ANGLE
    // stable hull 判断用的参数
  }
}

void LocalPlanning::LocalOctomapCallback(const octomap_msgs::Octomap& msg) {
  octomap::IgTree temp_ig_tree(0.5);
  std::lock_guard<std::mutex> lock_octo(local_octomap_mutex_);
  std::lock_guard<std::mutex> lock_srv0(service0_mutex_);
  std::lock_guard<std::mutex> lock_srv1(service1_mutex_);
  std::lock_guard<std::mutex> lock_srv2(service2_mutex_);
  std::lock_guard<std::mutex> lock_srv3(service3_mutex_);
  std::lock_guard<std::mutex> lock_srv4(service4_mutex_);
  std::lock_guard<std::mutex> lock_srv5(service5_mutex_);
  std::lock_guard<std::mutex> lock_srv6(service6_mutex_);
  std::lock_guard<std::mutex> lock_srv7(service7_mutex_);
  std::lock_guard<std::mutex> lock_srv8(service8_mutex_);

  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  if (aot) {
    if (!local_octree_ptr_) local_octree_ptr_.reset(new octomap::IgTree(0.025));
    local_octree_ptr_.reset(dynamic_cast<octomap::IgTree*>(aot));
  }
  // std::cout << "LocalPlanning::LocalOctomapCallback" << std::endl;
  if (costmap_enable_) {
    ros::WallTime t1 = ros::WallTime::now();
    LocalOctomapToElevationGrid();
    ros::WallDuration d = ros::WallTime::now() - t1;
    printf("<OctomapToElevationGrid>: timecost: %.3fs\n", d.toSec());
  }
  InitTrackSamples();
}

void LocalPlanning::GlobalPathCallback(const nav_msgs::PathConstPtr msg) {
  std::lock_guard<std::mutex> lock(local_octomap_mutex_);
  global_path_ = *msg;
  if (global_path_.poses.size() > 0) {
    global_path_flag_ = true;
  }
}

void LocalPlanning::LocalPoseArrayCallback(
    const geometry_msgs::PoseArrayConstPtr msg) {
  // global_path_ = *msg;
  global_path_.header = msg->header;
  global_path_.poses.clear();
  if (msg->poses.size() > 0) {
    for (int i = 0; i < msg->poses.size(); i++) {
      geometry_msgs::PoseStamped p;
      p.pose = msg->poses[i];
      global_path_.poses.push_back(p);
    }
    global_path_flag_ = true;
  }
}

void LocalPlanning::GenerateElevationMap(
    ElevationPointSet& elevation_map, pcl::PointCloud<pcl::PointXYZ>& cloud) {
  elevation_map.clear();
  for (int i = 0; i < cloud.size(); i++) {
    auto p = cloud.points[i];
    // Voxlized the pcd，存入时保存xyz，和2D key
    auto key = local_octree_ptr_->coordToKey(octomap::point3d(p.x, p.y, p.z));
    ElevationPoint ele_p3d(p.x, p.y, p.z, key);
    auto iter = elevation_map.find(ele_p3d);  // 避免重复，搜索时也用2D key
    if (iter == elevation_map.end()) {        // 不在elevation_map中
      elevation_map.insert(ele_p3d);
    } else if (ele_p3d.z > iter->z) {  // 在，并高于原来的点，替换之
      elevation_map.erase(iter);
      elevation_map.insert(ele_p3d);
    }
  }
}

Eigen::Vector3f LocalPlanning::GetAxisWithSinglePointContact(
    Eigen::Vector3f contact_point, Eigen::Vector3f track_center,
    Eigen::Vector3f gravity_transformed) {
  // 接触点->质心的向量，与重力方向的叉乘
  Eigen::Vector3f axis =
      (track_center - contact_point).cross(gravity_transformed);
  printf(
      "<LocalPlanning::GetAxisWithSinglePointContact>: axis = [%.3f, %.3f, "
      "%.3f]\n",
      axis.x(), axis.y(), axis.z());
  axis.normalize();
  return axis;
}

Eigen::Vector3f LocalPlanning::GetAxisWithTwoPointContact(
    std::pair<Eigen::Vector3f, Eigen::Vector3f> contact_points,
    Eigen::Vector3f track_center, Eigen::Vector3f gravity_transformed) {
  Eigen::Vector3f axis;
  auto p0 = contact_points.first;
  auto p1 = contact_points.second;
  Eigen::Vector3f diff = p0 - p1;
  if (diff.norm() > 0.25) {
    axis = GetCrossDir(p0, track_center, p1) ? diff : -diff;
    axis.normalize();
    printf(
        "<LocalPlanning::GetAxisWithTwoPointContact>: two far points. the line "
        "is axis\n");
    printf(
        "<LocalPlanning::GetAxisWithTwoPointContact>: axis = [%.3f, %.3f, "
        "%.3f]\n",
        axis.x(), axis.y(), axis.z());
  } else {
    Eigen::Vector3f mid = 0.5 * (p0 + p1);
    printf(
        "<LocalPlanning::GetAxisWithTwoPointContact>: two near points. merge "
        "into one.\n");
    axis =
        GetAxisWithSinglePointContact(mid, track_center, gravity_transformed);
  }
  return axis;
}

Eigen::Vector3f LocalPlanning::GetAxisWithHullEigenVector(
    pcl::PointCloud<pcl::PointXYZRGB>& hull_pcd, Eigen::Vector3f track_center) {
  // eigen, svd
  Eigen::Vector2f mean(0, 0);
  for (int i = 0; i < hull_pcd.size(); i++) {
    mean += Eigen::Vector2f(hull_pcd[i].x, hull_pcd[i].y);
  }
  mean /= hull_pcd.size();
  Eigen::MatrixXf A(hull_pcd.size(), 2);
  for (int i = 0; i < hull_pcd.size(); i++) {
    Eigen::Vector2f p = Eigen::Vector2f(hull_pcd[i].x, hull_pcd[i].y) - mean;
    A.row(i) = p;
  }
  Eigen::MatrixXf ATA = A.transpose() * A;
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(ATA, Eigen::ComputeFullV);
  svd.singularValues();

  Eigen::Vector2f normal = svd.matrixV().col(0);
  Eigen::Vector3f axis = Eigen::Vector3f(normal.x(), normal.y(), 0);
  axis.normalize();

  // correct axis direction
  // 适用于outside
  for (int i = 0; i < hull_pcd.size(); i++) {
    pcl::PointXYZRGB p0 = hull_pcd[i], p1 = hull_pcd[(i + 1) % hull_pcd.size()];
    Eigen::Vector3f pt0(p0.x, p0.y, p0.z), pt1(p1.x, p1.y, p1.z);
    if (GetCrossDir(pt0, track_center, pt1)) {
      if ((pt1 - pt0).dot(axis) < 0) {  // True: 与center同侧的向量
        axis = -axis;
      }
      break;
    }
  }
  // TODO(liyx) inside 处理

  printf(
      "<LocalPlanning::GetAxisWithMultiplePointContact>: axis = [%.3f, %.3f, "
      "%.3f]\n",
      axis.x(), axis.y(), axis.z());
  return axis;
}

Eigen::Vector3f LocalPlanning::GetAxisWithClosetHullEdge(
    pcl::PointCloud<pcl::PointXYZRGB>& hull_pcd, Eigen::Vector3f track_center) {
  float min_dis = 1e9;
  Eigen::Vector2f axis_2d(0, 0);
  Eigen::Vector2f center(track_center.x(), track_center.y());
  for (int i = 0; i < hull_pcd.size(); i++) {
    pcl::PointXYZRGB p0 = hull_pcd[i], p1 = hull_pcd[(i + 1) % hull_pcd.size()];
    Eigen::Vector2f pt0(p0.x, p0.y), pt1(p1.x, p1.y);

    // when center is outside hull, at least one candidate edge satisfy this
    // if (GetCrossDir(pt0, track_center, pt1))
    // {
    //  check the distance from track_center to the edge
    Eigen::Vector2f diff = pt1 - pt0;
    Eigen::Vector2f cross_point = PointToLine2D(center, pt1, pt0);
    float dis = (cross_point - center).norm();
    // std::cout << std::endl
    //           << "pt" << i << "=[" << pt0.transpose() << "]， "
    //           << "pt" << i + 1 << "=[" << pt1.transpose() << "], "
    //           << "len=" << diff.norm() << std::endl
    //           << "cross=[" << cross_point.transpose() << "], "
    //           << "center=[" << center.transpose() << "], "
    //           << "dis=" << dis << std::endl;

    if (diff.norm() < 0.1) continue;
    // select the edge closest to track_center as axis
    if (dis < min_dis) {
      min_dis = dis;
      axis_2d = diff;
    }
    // }
  }
  Eigen::Vector3f axis(axis_2d.x(), axis_2d.y(), 0);

  printf(
      "<LocalPlanning::GetAxisWithClosetHullEdge>: axis = [%.3f, %.3f, %.3f]\n",
      axis.x(), axis.y(), axis.z());
  axis.normalize();
  return axis;
}

// -1: error, contact_type: 0-stableHull, 1-singlePoint, 2-twoPoints,
// 3-smallHull, 4-MiddleHul
int LocalPlanning::SearchTrackContactAndRotation(
    NeighDomain& neigh_domain, SampleSet& track_samples, float diff_tolerance,
    float rotate_step, Eigen::Quaternionf pre_rotation,
    Eigen::Quaternionf& rotation_delta, TempVariableSet& variable_set) {
  if (neigh_domain.decentralized_points.size() == 0) {
    ROS_ERROR(
        "<LocalPlanning::SearchTrackContactAndRotation>: no neighbor points.");
    return -1;
  }

  // Rotate the neighbor points into local frame with normal vector alising
  // z-axis
  pcl::PointCloud<pcl::PointXYZ> cloud_trans;
  Eigen::Matrix3f rotation_matrix_inv =
      neigh_domain.normal_rotation.toRotationMatrix().transpose() *
      pre_rotation.toRotationMatrix().transpose();
  Eigen::Matrix3f rotation_matrix = rotation_matrix_inv.transpose();
  Eigen::Isometry3f T_inv = Eigen::Isometry3f::Identity();
  T_inv.rotate(rotation_matrix_inv);
  pcl::transformPointCloud(neigh_domain.decentralized_points, cloud_trans,
                           T_inv.matrix());

  // Construct elevation map and keep unique points (non-competitive)
  ElevationPointSet elevation_map;
  GenerateElevationMap(elevation_map, cloud_trans);
  // printf(
  //     "<LocalPlanning::SearchTrackContactAndRotation>: elevation_map SIZE =
  //     %d "
  //     "/ %d\n",
  //     static_cast<int>(elevation_map.size()),
  //     static_cast<int>(cloud_trans.size()));

  // Segment pcd under the tracks or the body
  float max_height_track = -1e6, max_height_body = -1e6;
  std::vector<ElevationPoint> points_under_track, points_under_body;

  // 筛选出track_samples在elevation_map中的点
  for (auto p : track_samples.track_samples) {
    auto key = local_octree_ptr_->coordToKey(
        octomap::point3d(p.x, p.y, p.z));  // 3D key
    auto iter =
        elevation_map.find(ElevationPoint(p.x, p.y, p.z, key));  // 搜索2D key
    if (iter != elevation_map.end()) {                           // 不在时为end
      // 用elevation_map中的对应点，然而没有去重
      points_under_track.push_back(*iter);
      max_height_track =
          std::max(max_height_track, static_cast<float>(iter->z));
    }
  }
  if (points_under_track.size() == 0) {
    ROS_ERROR(
        "<LocalPlanning::SearchTrackContactAndRotation>: No "
        "points_under_track");
    return -1;
  }
  // printf(
  //     "<LocalPlanning::SearchTrackContactAndRotation>: points_under_track: "
  //     "= %d / %d\n",
  //     static_cast<int>(points_under_track.size()),
  //     static_cast<int>(track_samples.track_samples.size()));

  // points_under_body
  for (auto p : track_samples.body_samples) {
    auto key = local_octree_ptr_->coordToKey(octomap::point3d(p.x, p.y, p.z));
    auto iter = elevation_map.find(ElevationPoint(p.x, p.y, p.z, key));
    if (iter != elevation_map.end()) {
      points_under_body.push_back(*iter);
      max_height_body = std::max(max_height_body, static_cast<float>(iter->z));
    }
  }

  // 判断body下的最大高度与履带最大高度的差，超过阈值则返回-1
  float max_height_dis = max_height_body - max_height_track;
  // printf(
  //     "<LocalPlanning::SearchTrackContactAndRotation>: body_height=%.3f, "
  //     "track_height=%.3f, dis=%.3f\n",
  //     max_height_body, max_height_track, max_height_dis);
  if (max_height_dis > 0.06 && strict_contact_) {  // trick: 0
    ROS_ERROR(
        "<LocalPlanning::SearchTrackContactAndRotation>: points_under_base "
        "collision!!!");
    return -1;
  }

  // Filter the pcd with height
  std::vector<ElevationPoint> track_contact_points;
  octomap::KeySet keyset_under_track;  // 用于去重
  // pcl::PointCloud<pcl::PointXYZRGB> cloud_no_contact;
  for (int i = 0; i < points_under_track.size(); i++) {
    auto p = points_under_track[i];
    if (keyset_under_track.find(p.k2d) == keyset_under_track.end()) {  // 去重
      keyset_under_track.insert(p.k2d);
      if (fabs(p.z - max_height_track) < diff_tolerance) {
        track_contact_points.push_back(p);
      } else {
        // pcl::PointXYZRGB pc(255, 255, 0);
        // pc.x = p.x, pc.y = p.y, pc.z = max_height_track;  // 无需真实z
        // cloud_no_contact.push_back(pc);
      }
    }
  }
  // printf(
  //     "<LocalPlanning::SearchTrackContactAndRotation>: track_contact_points "
  //     "SIZE = %d\n",
  //     static_cast<int>(track_contact_points.size()));

  pcl::PointCloud<pcl::PointXYZRGB> cloud_contact, cloud_contact_trans,
      hull_pcd;
  Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
  T.rotate(rotation_matrix);
  T.pretranslate(neigh_domain.base_center);  // mass_center -> base_center
  Eigen::Vector3f gravity_transformed =
      T.rotation() * Eigen::Vector3f(0, 0, -1);
  Eigen::Vector3f track_center(0, 0, max_height_track);
  Eigen::Vector3f axis(0, 0, 0);

  ContactType contact_type;
  float hull_area = 0;
  //  Determine rotational axis
  switch (track_contact_points.size()) {
    case 0: {
      ROS_ERROR(
          "<LocalPlanning::SearchTrackContactAndRotation>: No "
          "track_contact_points");
      return -1;
    } break;

    case 1: {
      // printf(
      //     "<LocalPlanning::SearchTrackContactAndRotation>: Single "
      //     "track_contact_points\n");
      pcl::PointXYZRGB p(0, 255, 0);
      p.x = track_contact_points[0].x;
      p.y = track_contact_points[0].y;
      p.z = track_contact_points[0].z;
      cloud_contact.push_back(p);
      axis = GetAxisWithSinglePointContact(Eigen::Vector3f(p.x, p.y, p.z),
                                           track_center, gravity_transformed);
      contact_type = SinglePoint;
    } break;

    case 2: {
      // printf(
      //     "<LocalPlanning::SearchTrackContactAndRotation>: Two "
      //     "track_contact_points\n");
      for (int i = 0; i < track_contact_points.size(); i++) {
        pcl::PointXYZRGB p(0, 255, 0);
        p.x = track_contact_points[0].x;
        p.y = track_contact_points[0].y;
        p.z = track_contact_points[0].z;
        cloud_contact.push_back(p);
      }
      // 去中心，按tan排序，排序后为逆时针方向
      pcl::PointXYZRGB p0 = cloud_contact.points[0],
                       p1 = cloud_contact.points[1];
      auto point_pair = std::make_pair(Eigen::Vector3f(p0.x, p0.y, p0.z),
                                       Eigen::Vector3f(p1.x, p1.y, p1.z));
      axis = GetAxisWithTwoPointContact(point_pair, track_center,
                                        gravity_transformed);
      contact_type = TwoPoints;
    } break;

    default: {
      // Call QuickHull with contact_points, sort inside.
      QuickHull2D<ElevationPoint> qh(track_contact_points);
      auto hull_pointvec =
          qh.getConvexHull_Qhull();  // 返回2D vector，需要查询高程图
      if (hull_pointvec.size() == 0) {
        ROS_ERROR(
            "<LocalPlanning::SearchTrackContactAndRotation>: No pointset_hull");
        return -1;
      }
      ElevationPointSet hull_pointset;

      for (auto p : hull_pointvec) {  // 需要查询高程图，获取3D点
        auto key = local_octree_ptr_->coordToKey(octomap::point3d(p.x, p.y, 0));
        auto iter = elevation_map.find(ElevationPoint(p.x, p.y, 0, key));
        if (iter != elevation_map.end()) {  // 在elevation_map中
          hull_pointset.insert(*iter);
          pcl::PointXYZRGB pc(255, 0, 0);
          pc.x = iter->x, pc.y = iter->y, pc.z = iter->z;
          hull_pcd.push_back(pc);
        }
      }
      if (hull_pointset.size() == 0) {
        ROS_ERROR(
            "<LocalPlanning::SearchTrackContactAndRotation>: No hull_pointset");
        return -1;
      }
      // printf(
      //     "<LocalPlanning::SearchTrackContactAndRotation>: hull_pointset SIZE
      //     "
      //     "= %d\n",
      //     static_cast<int>(hull_pointset.size()));

      //  Determine rotational axis
      switch (hull_pcd.size()) {
        case 1: {
          // printf(
          //     "<LocalPlanning::SearchTrackContactAndRotation>: Convex hull, "
          //     "single vertex.\n");
          auto p = hull_pcd[0];
          axis =
              GetAxisWithSinglePointContact(Eigen::Vector3f(p.x, p.y, p.z),
                                            track_center, gravity_transformed);
          contact_type = SinglePoint;
        } break;

        case 2: {
          // printf(
          //     "<LocalPlanning::SearchTrackContactAndRotation>: Convex hull, "
          //     "two vertices.\n");
          pcl::PointXYZRGB p0 = hull_pcd[0], p1 = hull_pcd[1];
          auto point_pair = std::make_pair(Eigen::Vector3f(p0.x, p0.y, p0.z),
                                           Eigen::Vector3f(p1.x, p1.y, p1.z));
          axis = GetAxisWithTwoPointContact(point_pair, track_center,
                                            gravity_transformed);
          contact_type = TwoPoints;
        } break;

        default: {
          // Calculation hull area %
          hull_area = qh.getHullArea() / track_samples.track_area;
          bool inHull = qh.inConvexPoly(
              ElevationPoint(track_center.x(), track_center.y(), 0));
          // printf(
          //     "<LocalPlanning::SearchTrackContactAndRotation>: hull_area = "
          //     "%.3f, inHull = %d\n",
          //     hull_area, inHull);

          if (hull_area > 0.2) {    // 0.10 / track_samples.track_area
            if (hull_area > 0.4 &&  // 0.15 / track_samples.track_area
                inHull == true) {
              // printf(
              //     "<LocalPlanning::SearchTrackContactAndRotation>: Big hull "
              //     "inside, stable contact.\n");
              axis = Eigen::Vector3f(0, 0, 1);  // point upward
              contact_type = StableHull;
            } else {
              // printf(
              //     "<LocalPlanning::SearchTrackContactAndRotation>: Big hull "
              //     "outside, select closest edge.\n");
              axis = GetAxisWithClosetHullEdge(hull_pcd, track_center);
              contact_type = MiddleHull;
            }
          } else {
            // 也有可能是单点
            if (hull_area < 0.1) {  // 0.05 / track_samples.track_area
              // printf(
              //     "<LocalPlanning::SearchTrackContactAndRotation>: Tiny hull,
              //     " "treat as single point.\n");
              Eigen::Vector4d hull_center;
              pcl::compute3DCentroid(hull_pcd, hull_center);
              axis = GetAxisWithSinglePointContact(
                  Eigen::Vector3f(hull_center.x(), hull_center.y(),
                                  hull_center.z()),
                  track_center, gravity_transformed);
              contact_type = SinglePoint;
            } else {
              // printf(
              //     "<LocalPlanning::SearchTrackContactAndRotation>: Small
              //     hull, " "select max eigen vector.\n");
              axis = GetAxisWithHullEigenVector(hull_pcd, track_center);
              contact_type = SmallHull;
            }
          }
        } break;
      }

      // Color the contact_points for visulization
      for (auto p : track_contact_points) {
        pcl::PointXYZRGB pc(0, 0, 255);  // 蓝色：邻域接触点（凸包外）
        if (hull_pointset.find(p) != hull_pointset.end())  // 红色：凸包点
          pc = pcl::PointXYZRGB(255, 0, 0);
        else if (qh.inConvexPoly(p))  // 凸包内：绿色
          pc = pcl::PointXYZRGB(0, 255, 0);
        pc.x = p.x, pc.y = p.y, pc.z = p.z;
        cloud_contact.push_back(pc);
      }
    }
  }
  // printf(
  //     "<LocalPlanning::SearchTrackContactAndRotation>: Rotate axis = [%.3f, "
  //     "%.3f, %.3f]\n",
  //     axis.x(), axis.y(), axis.z());

  // Transform to world frame for visulization
  Eigen::Vector3f track_center_trans =
      T.rotation() * track_center + T.translation();
  Eigen::Vector3f axis_trans = T.rotation() * axis;

  pcl::PointCloud<pcl::PointXYZRGB> hull_pcd_trans;
  if (hull_pcd.size() != 0) {
    pcl::transformPointCloud(hull_pcd, hull_pcd_trans, T.matrix());
  }
  // PublishConvexHullMarkers(hull_pcd_trans, track_center_trans);
  // else {
  //   ClearConvexHullMarkers();
  // }

  // publish marker of axis_trans
  // PublishVectorPoseMarker(track_rotation_axis_publisher_, track_center_trans,
  //                         axis_trans);

  // Elevate track boundries with max_height_track
  pcl::PointCloud<pcl::PointXYZRGB> track_boundries, track_boundries_trans;
  for (int i = 0; i < track_samples.track_corners.size(); i++) {
    pcl::PointXYZRGB p(255, 255, 0);
    p.x = track_samples.track_corners[i].x;
    p.y = track_samples.track_corners[i].y;
    p.z = track_samples.track_corners[i].z + max_height_track;
    track_boundries.push_back(p);
  }

  // Visualize track boundries
  pcl::transformPointCloud(track_boundries, track_boundries_trans, T.matrix());
  // PublishTrackBoundaryMarkers(track_boundries_trans);  // Markers

  // Visualize all cloud
  pcl::transformPointCloud(cloud_contact, cloud_contact_trans, T.matrix());
  cloud_contact_trans += track_boundries_trans;
  // PublishColorPointcloud(track_contact_pcd_publisher_, cloud_contact_trans);

  variable_set = TempVariableSet(
      Eigen::Quaternionf(rotation_matrix), neigh_domain.base_center,
      neigh_domain.normal_vector, max_height_track, diff_tolerance, hull_area,
      hull_pcd, cloud_contact_trans, elevation_map, track_center_trans,
      axis_trans, hull_pcd_trans, track_boundries_trans);

  if (contact_type == StableHull) {
    rotation_delta = Eigen::Quaternionf::Identity();
    // printf(
    //     "<LocalPlanning::SearchTrackContactAndRotation>: Single round "
    //     "completed!\n");
    return contact_type;
  }

  rotation_delta =
      Eigen::Quaternionf(Eigen::AngleAxisf(M_PI * rotate_step / 180.0, axis));
  // printf(
  //     "<LocalPlanning::SearchTrackContactAndRotation>: Delta q = [%.3f, %.3f,
  //     "
  //     "%.3f, %.3f]\n",
  //     rotation_delta.x(), rotation_delta.y(), rotation_delta.z(),
  //     rotation_delta.w());

  return contact_type;
}

int LocalPlanning::SearchFlipperAngle(
    TempVariableSet& variable_set,  // 本次迭代的结果
    pcl::PointCloud<pcl::PointXYZ>& flipper_corners,
    pcl::PointCloud<pcl::PointXYZ>& flipper_samples, float flipper_length,
    Eigen::Quaternionf base_dir, bool enable, float* flipper_angle,
    pcl::PointCloud<pcl::PointXYZRGB>& flipper_corners_trans,
    pcl::PointCloud<pcl::PointXYZRGB>& cloud_contact_trans) {
  // 计算旋转轴
  Eigen::Vector2f flipper_axis_p0(flipper_corners.points[0].x,
                                  flipper_corners.points[0].y);
  Eigen::Vector2f flipper_axis_p1(flipper_corners.points[1].x,
                                  flipper_corners.points[1].y);
  Eigen::Vector3f axis_p0 =  // 对基准点旋转-yaw
      base_dir.inverse() *
      Eigen::Vector3f(flipper_axis_p0.x(), flipper_axis_p0.y(), 0);
  if (axis_p0.x() > 0) {  // 前侧的旋转轴反向
    std::swap(flipper_axis_p0, flipper_axis_p1);
  }
  Eigen::Vector2f axis_2d = flipper_axis_p1 - flipper_axis_p0;
  axis_2d.normalize();

  // 将新的接触点，存入hull_pcd，重新计算凸包 hull_pcd
  // auto hull_pcd = variable_set.hull_pcd;

  // 找到flipper下面的点，计算角度
  float max_tan = -1e3;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_contact;
  bool is_floating = false;
  if (enable) {
    octomap::KeySet key_set;
    using PointAnglePair = std::pair<ElevationPoint, float>;
    std::vector<PointAnglePair> points_under_flipper;
    for (auto p : flipper_samples) {
      // 转化为栅格坐标
      auto key = local_octree_ptr_->coordToKey(octomap::point3d(p.x, p.y, 0));
      auto pt = local_octree_ptr_->keyToCoord(key);

      // 查找2D坐标对应的点
      auto iter = variable_set.elevation_map.find(
          ElevationPoint(pt.x(), pt.y(), 0, key));
      if (iter != variable_set.elevation_map.end()) {  // 找到了对应的3D点
        if (key_set.find(key) != key_set.end())        // 忽略重复的
          continue;
        key_set.insert(key);

        // 保存点，用于生成新的凸包
        // pcl::PointXYZRGB p(255, 255, 0);
        // p.x = iter->x, p.y = iter->y, p.z = iter->z;
        // hull_pcd.push_back(p);

        // 计算p到旋转轴的距离(2d)
        Eigen::Vector2f pt_2d(iter->x, iter->y);
        Eigen::Vector2f cross =
            PointToLine2D(pt_2d, flipper_axis_p0, flipper_axis_p1);
        float dis_2d = (pt_2d - cross).norm();
        float dis_h = iter->z - variable_set.max_height_track;
        float dis = sqrt(pow(dis_2d, 2) + pow(dis_h, 2));
        bool cross_dir = GetCrossDir(
            Eigen::Vector3f(flipper_axis_p0.x(), flipper_axis_p0.y(), 0),
            Eigen::Vector3f(iter->x, iter->y, 0),
            Eigen::Vector3f(flipper_axis_p1.x(), flipper_axis_p1.y(), 0));
        bool over_len = (dis > (flipper_length + 0.03) || dis_2d < 0.05);
        // printf(
        //     "<LocalPlanning::SearchFlipperAngle>: d=%.3f, h=%.3f,
        //     len=%.3f/%.3f, tan=%.3f, " "theta=%.3f, over_length=%d,
        //     cross_dir=%d\n", dis_2d, dis_h, dis, flipper_length, dis_h /
        //     dis_2d, atan(dis_h / dis_2d) * 180.0 / M_PI, over_len,
        //     cross_dir);

        // if (cross_dir) {
        //   pcl::PointXYZRGB pc(255, 0, 0);
        //   pc.x = iter->x, pc.y = iter->y, pc.z = iter->z;
        //   cloud_contact.push_back(pc);
        // } else {
        //   pcl::PointXYZRGB pc(0, 255, 0);
        //   pc.x = iter->x, pc.y = iter->y, pc.z = iter->z;
        //   cloud_contact.push_back(pc);
        // }

        if (over_len || cross_dir)  // 距离大于flipper长度，无法接触
          continue;

        float tan_theta;
        if (fabs(dis_2d) > 1e-3) {  // 避免分母为0
          tan_theta = dis_h / dis_2d;
        } else {                     // 到轴的距离很近
          if (fabs(dis_h) < 1e-3) {  // 垂直距离也很近
            tan_theta = 0;
          } else {
            tan_theta = dis_h > 0 ? 1e5 : -1e5;
          }
        }

        if (fabs(atan(tan_theta)) > 80 &&       // 接近垂直
            dis_h < 0 &&                        // flipper向下时避免扎进地里
            fabs(dis - flipper_length) < 0.03)  // 接触点必须接近flipper长度
          continue;
        points_under_flipper.push_back(std::make_pair(*iter, tan_theta));
      }
    }

    // 对points_under_flipper，按tan_theta排序
    std::sort(points_under_flipper.begin(), points_under_flipper.end(),
              [&](PointAnglePair& p1, PointAnglePair& p2) -> bool {
                return (p1.second) > (p2.second);  // 从大到小
              });

    // 角度最大的部分作为接触点，标绿
    if (points_under_flipper.size() > 0) {
      // printf("<LocalPlanning::SearchFlipperAngle>: Max: %f deg, Min: %f
      // deg\n",
      //        atan(points_under_flipper.front().second) * 180.0 / M_PI,
      //        atan(points_under_flipper.back().second) * 180.0 / M_PI);
      max_tan = points_under_flipper.begin()->second;
      for (int i = 0; i < points_under_flipper.size(); i++) {
        auto p = points_under_flipper[i];
        if (fabs(atan(p.second) - atan(max_tan)) < M_PI * 5.0 / 180.0) {
          pcl::PointXYZRGB pc(0, 0, 255);
          pc.x = p.first.x, pc.y = p.first.y, pc.z = p.first.z;
          cloud_contact.push_back(pc);
        } else {
          break;  // 提前结束
        }
      }
    } else {
      ROS_ERROR(
          "<LocalPlanning::SearchFlipperAngle>: No valid point under flipper, "
          "set to default floating angle -45 deg");
      max_tan = -FLIPPER_RESET_ANGLE;
      is_floating = true;
      // return -1;
    }

    // 生成新的凸包
    // std::vector<ElevationPoint> points;
    // for (auto p : hull_pcd.points) {
    //   ElevationPoint pt(p.x, p.y, 0);
    //   points.push_back(pt);
    // }
    // QuickHull2D<ElevationPoint> qh(points);
    // auto hull_points = qh.getConvexHull_Qhull();
    // float new_area = qh.getHullArea();
    // printf("<LocalPlanning::SearchFlipperAngle>: New area %.3f / %.3f",
    // new_area,
    //          variable_set.hull_area);
    // if (fabs(new_area - variable_set.hull_area) / variable_set.hull_area <
    //     0.1) {  // 凸包面积变化<10%
    //   ROS_ERROR("<LocalPlanning::SearchFlipperAngle>: insufficient flipper
    //   contact"); return -1;
    // }

    // 后摇臂，需要+180°
    *flipper_angle = axis_p0.x() > 0 ? atan(max_tan)            // front: >0
                                     : (atan(max_tan) + M_PI);  // rear: <0
  } else {
    // static int cnt = 0;
    // static int i = -90;
    // if (cnt++ == 2) {
    //   cnt = 0;
    //   i += 10;
    //   if (i > 90) i = -90;
    // }
    // max_tan = tan(M_PI * i / 180.0);
    // ROS_ERROR("<LocalPlanning::SearchFlipperAngle>: flipper angle: %d,
    // dir_ori: %.3f, %.3f", i,
    //           axis_p0.x(), axis_p0.y());

    // max_tan = tan(45 * M_PI / 180.0);
    // 直接采用flipper_angle的值
  }

  // 旋转flipper corner点云到该角度
  Eigen::AngleAxisf axis_angle(*flipper_angle,
                               Eigen::Vector3f(axis_2d.x(), axis_2d.y(), 0));

  pcl::PointCloud<pcl::PointXYZRGB> flipper_corners_rgb;
  ColorPointcloud(flipper_corners, 255, 255, 0, flipper_corners_rgb);

  // flipper_samples需要先去中心
  Eigen::Vector3f rot_center(flipper_corners.points[0].x,
                             flipper_corners.points[0].y,
                             flipper_corners.points[0].z);
  Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
  T.translate(-rot_center);  // decentralize
  pcl::transformPointCloud(flipper_corners_rgb, flipper_corners_trans,
                           T.matrix());

  // 旋转并加高度
  T = Eigen::Isometry3f::Identity();
  T.rotate(axis_angle);
  rot_center.z() += variable_set.max_height_track;
  T.pretranslate(rot_center);
  pcl::transformPointCloud(flipper_corners_trans, flipper_corners_rgb,
                           T.matrix());

  // 最后变换T到world
  T = Eigen::Isometry3f::Identity();
  T.rotate(variable_set.pose);
  T.pretranslate(variable_set.position);
  pcl::transformPointCloud(flipper_corners_rgb, flipper_corners_trans,
                           T.matrix());

  // contact点云只需要变换T
  if (cloud_contact.size() > 0)
    pcl::transformPointCloud(cloud_contact, cloud_contact_trans, T.matrix());

  // PublishFlipperBoundaryMarkers(flipper_corners_trans);
  // PublishColorPointcloud(flipper_contact_pcd_publisher_,
  // cloud_contact_trans);
  if (is_floating) return -1;
  return 1;
}

void LocalPlanning::testGetMinRect(
    pcl::PointCloud<pcl::PointXYZRGB>& hull_pcd_no_yaw) {
  std::vector<pcl::PointXYZRGB> hull_pcd;
  for (auto p : hull_pcd_no_yaw) hull_pcd.push_back(p);
  MinBoundingRectangle<pcl::PointXYZRGB> mbr(hull_pcd);
  float area = mbr.getMinimumBoundingRectangle();
  return;
}

void LocalPlanning::CheckTrackContact(
    Eigen::Quaternionf base_dir, pcl::PointCloud<pcl::PointXYZRGB>& hull_pcd,
    bool* left_contact, bool* right_contact, bool* front_contact,
    bool* rear_contact, float* bbox_area) {
  // 对hull_pcd旋转-yaw
  pcl::PointCloud<pcl::PointXYZRGB> hull_pcd_no_yaw;
  Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
  T.rotate(base_dir.inverse());
  pcl::transformPointCloud(hull_pcd, hull_pcd_no_yaw, T);

  *left_contact = false, *right_contact = false;
  *front_contact = false, *rear_contact = false;
  float center_x = 0, center_y = 0;
  for (int i = 0; i < hull_pcd_no_yaw.size(); i++) {
    // 判断左右履带有无接触，y>0 为右接触，<0 为左接触
    if (hull_pcd_no_yaw[i].y > 0)
      *left_contact = true;
    else
      *right_contact = true;
    center_x += hull_pcd_no_yaw[i].x;
    center_y += hull_pcd_no_yaw[i].y;
  }
  center_x /= hull_pcd_no_yaw.size();
  center_y /= hull_pcd_no_yaw.size();

  // 计算履带接触的最小包围框
  std::vector<pcl::PointXYZRGB> hull_points;
  for (auto p : hull_pcd_no_yaw) hull_points.push_back(p);
  MinBoundingRectangle<pcl::PointXYZRGB> mbr(hull_points);
  *bbox_area = mbr.getMinimumBoundingRectangle();

  if (fabs(center_x) > 0.1) {
    if (center_x > 0)  // 凸包中心在前，则前侧接触
      *front_contact = true;
    else
      *rear_contact = true;
  } else {  // 凸包中心很靠近底盘中心
    *front_contact = true;
    *rear_contact = true;
  }
}

int LocalPlanning::SearchFlipperContactAndRotation(
    std::vector<std::pair<TempVariableSet, int>>& variable_set_list,
    SampleSet& track_samples, ConfigurationPoint& way_point) {
  TempVariableSet variable_set =
      variable_set_list.back().first;  // selected configuration.
  float front_flipper_angle, rear_flipper_angle;

  bool left_contact = false, right_contact = false;
  bool front_contact = false, rear_contact = false;
  float max_area = 0, bbox_area = 0;
  Eigen::Vector3f inclination;

  bool isStableHull = false;
  if (variable_set_list.back().second ==
      StableHull) {  // track稳定，isStableHull设为True
    if (!CheckEulerConstraint(variable_set.pose, track_samples.pose,
                              inclination, 45, 45)) {
      ROS_ERROR(
          "<LocalPlanning::SearchFlipperContactAndRotation>: Case StableHull, "
          "slope is out of bound. Normal[%.3f,%.3f,%.3f]",
          variable_set.normal.x(), variable_set.normal.y(),
          variable_set.normal.z());
      return -1;
    }

    // 是否左右两脚同时接触，并计算履带接触凸包
    CheckTrackContact(track_samples.pose, variable_set.hull_pcd, &left_contact,
                      &right_contact, &front_contact, &rear_contact,
                      &bbox_area);
    max_area = variable_set.hull_area;
    printf(
        "<LocalPlanning::SearchFlipperContactAndRotation>: Case StableHull\n");
    isStableHull = true;
  } else {  // track不稳定，找到面积最大的
    // printf(
    //     "<LocalPlanning::SearchFlipperContactAndRotation>: Only SmallHull or
    //     " "MiddleHull\n");
    for (int i = 0; i < variable_set_list.size(); i++) {
      auto& set = variable_set_list[i];
      if (set.second == SmallHull || set.second == MiddleHull) {
        // printf(
        //     "<LocalPlanning::SearchFlipperContactAndRotation>: %d-th "
        //     "TrackContact: Area=%.3f\n",
        //     i, set.first.hull_area);
        if (!CheckEulerConstraint(set.first.pose, track_samples.pose,
                                  inclination, 45, 45)) {
          // ROS_ERROR(
          //     "<LocalPlanning::SearchFlipperContactAndRotation>: Case "
          //     "SmallHull or MiddleHull. Slope is out of bound. "
          //     "Normal[%.3f,%.3f,%.3f]",
          //     set.first.normal.x(), set.first.normal.y(),
          //     set.first.normal.z());
          continue;
        }

        CheckTrackContact(track_samples.pose, set.first.hull_pcd, &left_contact,
                          &right_contact, &front_contact, &rear_contact,
                          &bbox_area);
        if (left_contact && right_contact && set.first.hull_area > max_area) {
          variable_set = set.first;
          max_area = set.first.hull_area;
        }
      }
    }

    if (max_area == 0) {
      ROS_ERROR(
          "<LocalPlanning::SearchFlipperContactAndRotation>: No feasible track "
          "contact with max_area = 0. return -1");
      return -1;
    }
    printf(
        "<LocalPlanning::SearchFlipperContactAndRotation>: The max hull_area = "
        "%.3f\n",
        max_area);
    // } else {
    // // 若重心在中间，则启用前后双摇臂
    // if (front_contact && rear_contact) {
    //   flipper_rear_flag = true;
    //   flipper_front_flag = true;
    // } else {
    //   // 若重心在前，则启用后摇臂
    //   if (front_contact) {
    //     flipper_rear_flag = true;
    //   } else if (rear_contact) {
    //     // 若重心在后，则启用前摇臂
    //     flipper_front_flag = true;
    //   }
    // }
    // }
  }

  // Publish for track
  bool flipper_front_flag = true, flipper_rear_flag = true;
  PublishConvexHullMarkers(track_convex_hull_publisher_,
                           variable_set.hull_pcd_trans,
                           variable_set.track_center_trans);
  PublishVectorPoseMarker(track_rotation_axis_publisher_,
                          variable_set.track_center_trans,
                          variable_set.axis_trans);
  PublishTrackBoundaryMarkers(variable_set.track_boundries_trans);
  PublishColorPointcloud(track_contact_pcd_publisher_,
                         variable_set.cloud_contact_trans);

  // 根据flipper_front_flag、flipper_rear_flag判断是否进行规划摇臂
  pcl::PointCloud<pcl::PointXYZRGB> front_flipper_corners,
      front_flipper_contacts;
  int ret1 = SearchFlipperAngle(
      variable_set, track_samples.front_flipper_corners,
      track_samples.front_flipper_samples, track_samples.flipper_length,
      track_samples.pose, flipper_front_flag, &front_flipper_angle,
      front_flipper_corners, front_flipper_contacts);

  pcl::PointCloud<pcl::PointXYZRGB> rear_flipper_corners, rear_flipper_contacts;
  int ret2 = SearchFlipperAngle(
      variable_set, track_samples.rear_flipper_corners,
      track_samples.rear_flipper_samples, track_samples.flipper_length,
      track_samples.pose, flipper_rear_flag, &rear_flipper_angle,
      rear_flipper_corners, rear_flipper_contacts);

  pcl::PointCloud<pcl::PointXYZRGB> flipper_corners,
      flipper_contacts;  // world_frame

  flipper_corners += front_flipper_corners;
  flipper_contacts += front_flipper_contacts;
  flipper_corners += rear_flipper_corners;
  flipper_contacts += rear_flipper_contacts;
  std::pair<float, float> flipper_angles;
  flipper_angles.first = front_flipper_angle;
  flipper_angles.second = rear_flipper_angle - M_PI;  // 变回到±M_PI

  if (isStableHull) {               // 稳定构型，pitch<22.5时，收起摇臂(45°)
    float pitch = inclination.y();  // 抬头为负
    // 前后摇臂角度为|22.5|以上接触角度，则不执行
    if (fabs(flipper_angles.first) > FLIPPER_RESET_ANGLE * 0.5 ||
        fabs(flipper_angles.second) > FLIPPER_RESET_ANGLE * 0.5) {
      // Do nothing
    } else {
      if (fabs(pitch) < FLAT_PITCH_THRESHOLD) {
        // flipper_angles.first = FLIPPER_RESET_ANGLE;  // trick: 1
        // flipper_angles.second = FLIPPER_RESET_ANGLE;
        // printf(
        //     "<LocalPlanning::SearchFlipperContactAndRotation>: Flat ground, "
        //     "set both flippers to %.3f\n",
        //     FLIPPER_RESET_ANGLE * 180.0 / M_PI);
      }
    }
  }

  if (!isStableHull) {  // 为最大的凸包的构型，计算加上摇臂后的凸包，判断ZMP
                        // 如果ZMP满足，则接受；如果不满足，则返回-1
    pcl::PointCloud<pcl::PointXYZRGB> base_contacts, base_contacts_trans,
        base_contact_hull, base_contact_hull_trans;
    Eigen::Vector3f elevated_position =
        variable_set.position +
        Eigen::Vector3f(0, 0, variable_set.max_height_track);

    if (flipper_contacts.size() > 0) {
      base_contacts = flipper_contacts + variable_set.hull_pcd_trans;
      // printf(
      //     "<LocalPlanning::SearchFlipperContactAndRotation>:
      //     flipper_contacts: "
      //     "%d, track_hull_points: %d, base_contacts: %d\n",
      //     static_cast<int>(flipper_contacts.size()),
      //     static_cast<int>(variable_set.hull_pcd_trans.size()),
      //     static_cast<int>(base_contacts.size()));  // NOLINT

      // 变回到base坐标系
      Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
      T.rotate(variable_set.pose);
      T.pretranslate(elevated_position);
      pcl::transformPointCloud(base_contacts, base_contacts_trans,
                               T.matrix().inverse());
      std::vector<pcl::PointXYZRGB> base_contact_points;
      for (auto p : base_contacts_trans) base_contact_points.push_back(p);

      // 求凸包面积
      QuickHull2D<pcl::PointXYZRGB> qh(base_contact_points);
      std::vector<pcl::PointXYZRGB> hull_points = qh.getConvexHull_Qhull();
      float base_hull_area = qh.getHullArea() / track_samples.track_area;

      // 检查ZMP
      pcl::PointXYZRGB center;
      center.x = 0;
      center.y = 0;
      center.z = 0;
      bool inHull = qh.inConvexPoly(center);

      // 将凸包顶点变换到world坐标系，用于可视化
      for (auto p : hull_points) base_contact_hull.push_back(p);
      pcl::transformPointCloud(base_contact_hull, base_contact_hull_trans,
                               T.matrix());

      PublishConvexHullMarkers(base_convex_hull_publisher_,
                               base_contact_hull_trans, elevated_position);
      float ratio_th = 0.6;  // 只在track不稳定时执行，阈值可以大一些
      // printf(
      //     "<LocalPlanning::SearchFlipperContactAndRotation>: original area: "
      //     "%.3f, final contact area: %.3f / %.3f\n",
      //     variable_set.hull_area, base_hull_area, ratio_th);
      if (base_hull_area < ratio_th && strict_contact_) {
        ROS_ERROR(
            "<LocalPlanning::SearchFlipperContactAndRotation>: final contact "
            "is insufficient, return -1!!!");
        return -1;  // trick: 2
      }
      if (!inHull && strict_contact_) {
        ROS_ERROR(
            "<LocalPlanning::SearchFlipperContactAndRotation>: ZMP is still "
            "out of the new hull with flippers, return -1!!!");
        return -1;  // trick: 3
      }
    } else {
      PublishConvexHullMarkers(base_convex_hull_publisher_,
                               base_contact_hull_trans, elevated_position);
    }

    if (ret1 != 1 && ret2 != 1 && strict_contact_) {
      ROS_ERROR(
          "<LocalPlanning::SearchFlipperContactAndRotation>: inStableHull with "
          "two floating flippers, return -1!!!");
      return -1;  // trick: 4
    }
  }

  way_point.flipper_angles = flipper_angles;
  way_point.offset = Eigen::Vector3f(
      0, 0, variable_set.track_center_trans.z() - way_point.position.z());
  way_point.pose = variable_set.pose * track_samples.pose;

  if (flipper_corners.size() > 0)
    PublishFlipperBoundaryMarkers(flipper_corners);

  if (flipper_contacts.size() > 0)
    PublishColorPointcloud(flipper_contact_pcd_publisher_, flipper_contacts);

  printf(
      "<LocalPlanning::SearchFlipperContactAndRotation>: Flipper contact angle "
      "(%.3f, %.3f).\n",
      flipper_angles.first * 180.0 / M_PI,
      flipper_angles.second * 180.0 / M_PI);
  return 1;
}

bool LocalPlanning::PlanConfiguration(ConfigurationPoint& way_point,
                                      Eigen::Vector3f* goal_normal) {
  Eigen::Vector3f goal_pos = way_point.position;
  Eigen::Quaternionf goal_dir = way_point.orientation;
  // 1. estimate normal vector with local voxels v
  NeighDomain neigh_domain;
  neigh_domain.base_dir = goal_dir;

  SampleSet current_track_samples(goal_dir, initial_track_samples_);

  ros::WallTime t0 = ros::WallTime::now();
  if (EstimateNormalVector(goal_pos, 0.6, neigh_domain,
                           &current_track_samples) == false) {
    ROS_ERROR(
        "<LocalPlanning::EstimateNormalVector>: Inclication is too large, "
        "return false");
    return false;
  }

  auto goal_norm_vector = neigh_domain.normal_vector;
  printf(
      "<LocalPlanning::PlanConfiguration>: Normal vector: [%.3f, %.3f, %.3f], "
      "timecost: %.3fms\n",
      goal_norm_vector.x(), goal_norm_vector.y(), goal_norm_vector.z(),
      (ros::WallTime::now() - t0).toSec() * 1000);

  // transform track samples with goal_dir(yaw)

  // 2. SearchTrackContactAndRotation
  int status = 0, cnt = 0;
  Eigen::Quaternionf pre_rotation = Eigen::Quaternionf::Identity();
  Eigen::Quaternionf rotation_delta = Eigen::Quaternionf::Identity();
  std::vector<std::pair<TempVariableSet, int>> variable_set_list;
  do {
    ros::WallTime t1 = ros::WallTime::now();
    TempVariableSet variable_set;
    status = SearchTrackContactAndRotation(
        neigh_domain, current_track_samples,
        local_octree_ptr_->getResolution() * 3, 5.0, pre_rotation,
        rotation_delta, variable_set);
    pre_rotation *= rotation_delta;
    variable_set_list.push_back(std::make_pair(variable_set, status));
    // printf(
    //     "<LocalPlanning::PlanConfiguration>: Find %d th track contact with "
    //     "type %d, timecost %.2fms\n",
    //     cnt, status, (ros::WallTime::now() - t1).toSec() * 1000);
    if (status == -1 && variable_set_list.size() == 1) {  // 没有可行的
      way_point.timecost = ros::WallTime::now() - t1;
      way_point.pose = variable_set.pose;
      ROS_ERROR(
          "<LocalPlanning::PlanConfiguration>: No feasible base contact, "
          "return false");
      return false;
    }
  } while (status > 0 && ros::ok() && cnt++ < 10);

  // 3. SearchFlipperContactAndRotation
  ros::WallTime t2 = ros::WallTime::now();
  std::pair<float, float> flipper_angles;
  status = SearchFlipperContactAndRotation(variable_set_list,
                                           current_track_samples, way_point);
  ros::WallDuration tc = ros::WallTime::now() - t2;
  way_point.timecost = tc;
  if (status == -1) {
    ROS_ERROR(
        "<LocalPlanning::PlanConfiguration>: No feasible flipper contact, "
        "return false");
    return false;
  }

  printf(
      "<LocalPlanning::PlanConfiguration>: Get feasible configuration, "
      "timecost = %.2fms, return true\n",
      tc.toSec() * 1000);

  // geometry_msgs::Twist msg;
  // msg.linear.x = flipper_angles.first;
  // msg.linear.y = flipper_angles.first;
  // msg.angular.x = flipper_angles.second;
  // msg.angular.y = flipper_angles.second;
  // flipper_angle_publisher_.publish(msg);

  return true;
}

// 先给个3D点
void LocalPlanning::GoalPosCallback(
    const geometry_msgs::PointStampedConstPtr msg) {
  // printf("<LocalPlanning::goalPosCallback>: %.3f, %.3f, %.3f\n",
  // msg->point.x, msg->point.y, msg->point.z);
  goal_pos_flag_ = true;
  goal_pos_ = Eigen::Vector3f(msg->point.x, msg->point.y, msg->point.z);
  goal_dir_flag_ = false;
}

// 再给个方向，然后开始规划
void LocalPlanning::GoalDirCallback(
    const geometry_msgs::PoseStampedConstPtr msg) {
  //  2d nav goal只能选2d点，用clicked_point选择3d点
  printf("<LocalPlanning::goalDirCallback>: goal_pos=[%.3f, %.3f, %.3f]\n",
         goal_pos_.x(), goal_pos_.y(), goal_pos_.z());
  printf(
      "<LocalPlanning::goalDirCallback>: goal_dir=[%.3f, %.3f, %.3f, %.3f]\n",
      msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z);
  if (goal_pos_flag_ == false) {
    return;
  } else {
    goal_dir_flag_ = true;
    goal_dir_ =
        Eigen::Quaternionf(msg->pose.orientation.w, msg->pose.orientation.x,
                           msg->pose.orientation.y, msg->pose.orientation.z);

    // merge goal_dir and goal_pos into a 3D nav goal
    geometry_msgs::PoseStamped goal_3d_msg;
    goal_3d_msg = *msg;
    goal_3d_msg.pose.position.x = goal_pos_.x();
    goal_3d_msg.pose.position.y = goal_pos_.y();
    goal_3d_msg.pose.position.z = goal_pos_.z();
    goal_3d_publisher_.publish(goal_3d_msg);
  }

  if (!local_octree_ptr_) {
    ROS_ERROR(
        "<queryContactConfigurationServerCallback>: "
        "octree is null, return false");
    return;
  }
  ConfigurationPoint way_point;
  way_point.position = goal_pos_;
  way_point.orientation = goal_dir_;
  PlanConfiguration(way_point);

  // reset flags
  goal_dir_flag_ = false;
  goal_pos_flag_ = false;
}

// 用于测试为odom所在位置，规划构型
void LocalPlanning::SinglePointPlanning(geometry_msgs::Point p,
                                        geometry_msgs::Quaternion q) {
  octomap::point3d p3d(p.x, p.y, p.z - 1);  // z向下取1m
  // 从下向上(-1 ~ 1)，找到最高的体素。为什么不是从上向下找???
  std::vector<octomap::OcTreeKey> occ_keys;
  octomap::OcTreeKey k = local_octree_ptr_->coordToKey(p3d);
  for (int i = 0; i < 2.0 / local_octree_ptr_->getResolution(); i++) {
    octomap::OcTreeKey key;
    key.k[0] = k[0];
    key.k[1] = k[1];
    key.k[2] = k[2] + i;
    auto coord = local_octree_ptr_->keyToCoord(key);
    octomap::IgTreeNode* node = local_octree_ptr_->search(key);
    if (node && local_octree_ptr_->isNodeOccupied(node)) {
      occ_keys.push_back(key);
    }
  }
  if (occ_keys.size() > 0) {
    k = occ_keys.back();
    p3d = local_octree_ptr_->keyToCoord(k);
    float yaw = tf::getYaw(q);

    geometry_msgs::PoseStamped goal_3d_msg;
    goal_3d_msg.header.frame_id = map_frame_;
    goal_3d_msg.header.stamp = ros::Time::now();
    goal_3d_msg.pose.position.x = p3d.x();
    goal_3d_msg.pose.position.y = p3d.y();
    goal_3d_msg.pose.position.z = p3d.z();
    geometry_msgs::Quaternion q_yaw =
        tf::createQuaternionMsgFromYaw(yaw);  // 只用yaw角
    goal_3d_msg.pose.orientation = q_yaw;
    goal_3d_publisher_.publish(goal_3d_msg);

    ros::WallTime t = ros::WallTime::now();
    ConfigurationPoint way_point;
    way_point.position = Eigen::Vector3f(p3d.x(), p3d.y(), p3d.z());
    way_point.orientation =
        Eigen::Quaternionf(q_yaw.w, q_yaw.x, q_yaw.y, q_yaw.z);
    PlanConfiguration(way_point);
    ros::WallDuration d = ros::WallTime::now() - t;
    printf("<LocalPlanning::singlePointPlanning>: Planning timecost = %.4f\n",
           d.toSec());
  } else {
    printf(
        "<LocalPlanning::singlePointPlanning>: No valid occ_keys at current "
        "pose\n");
  }
}

// 用于测试为odom所在位置，规划构型
void LocalPlanning::OdomCallback(const nav_msgs::OdometryConstPtr msg) {
  SinglePointPlanning(
      msg->pose.pose.position,
      msg->pose.pose.orientation);  // 用于测试为odom所在位置，规划构型
}

void LocalPlanning::Run() {
  bool flag = false;
  {
    std::lock_guard<std::mutex> lock(local_octomap_mutex_);
    flag = global_path_flag_;
  }
  if (flag) {
    std::lock_guard<std::mutex> lock(local_octomap_mutex_);
    Search(global_path_, lookahead_distance_, true);
    global_path_flag_ = false;
  }
}

float LocalPlanning::GetOrientationBetweenTwoWaypoints(
    geometry_msgs::Point p1, geometry_msgs::Point p2) {
  Eigen::Vector3f dis(p2.x - p1.x, p2.y - p1.y, 0);
  Eigen::Vector3f euler = toEulerAngle(
      Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(), dis));
  return euler.z();
}

void LocalPlanning::PublishLocalPath(
    std::vector<ConfigurationPoint>& waypoints) {
  nav_msgs::Path local_path;
  geometry_msgs::PoseArray pose_array;
  for (int k = 0; k < waypoints.size(); k++) {
    geometry_msgs::PoseStamped p;  // = path.poses.at(k);
    p.pose.position.x = waypoints[k].position.x();
    p.pose.position.y = waypoints[k].position.y();
    p.pose.position.z = waypoints[k].position.z() + waypoints[k].offset.z();
    p.pose.orientation.w = waypoints[k].pose.w();
    p.pose.orientation.x = waypoints[k].pose.x();
    p.pose.orientation.y = waypoints[k].pose.y();
    p.pose.orientation.z = waypoints[k].pose.z();
    local_path.poses.push_back(p);  // for visualization

    p.pose.orientation.x = waypoints[k].flipper_angles.first;
    p.pose.orientation.y = waypoints[k].flipper_angles.second;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 0;
    pose_array.poses.push_back(p.pose);  // for path follower
  }

  local_path.header.frame_id = map_frame_;
  local_path.header.stamp = ros::Time::now();
  pose_array.header = local_path.header;
  path_publisher_.publish(local_path);
  pose_array_publisher_.publish(pose_array);
}

// 给定路径，路点近邻搜索半径，返回4维的路点序列（x、y、Θ1、Θ2）
bool LocalPlanning::Search(nav_msgs::Path& path, float max_dis, bool intepolate,
                           std::vector<ConfigurationPoint>* wps_ptr) {
  ros::WallTime t1 = ros::WallTime::now();
  if (path.poses.size() <= 1) {
    ROS_ERROR("<LocalPlanning::search>: path is null, return");
    return false;
  }
  if (!local_octree_ptr_) {
    ROS_ERROR("<LocalPlanning::search>: octree is null, return");
    return false;
  }

  if (intepolate) {  // 路点间插值
    nav_msgs::Path tmp = path;
    path.poses.clear();
    for (int i = 0; i < tmp.poses.size() - 1; i++) {
      path.poses.push_back(tmp.poses[i]);
      auto p0 = tmp.poses[i].pose.position;
      auto p1 = tmp.poses[i + 1].pose.position;

      geometry_msgs::PoseStamped p;
      p.pose.position.x = (p0.x + p1.x) * 0.5;
      p.pose.position.y = (p0.y + p1.y) * 0.5;
      p.pose.position.z = (p0.z + p1.z) * 0.5;
      path.poses.push_back(p);
    }
  }
  std::cout << "<LocalPlanning::search>: Octree Resolution = "
            << local_octree_ptr_->getResolution() << ", Init track_samples = "
            << initial_track_samples_.track_samples.size() << std::endl;

  float max_index = path.poses.size();
  for (int i = 0; i < path.poses.size(); i++) {
    geometry_msgs::Point p = path.poses[i].pose.position;
    if (i > 0) {
      auto p0 = path.poses[i - 1].pose.position;
      Eigen::Vector3f dis(p0.x - p.x, p0.y - p.y, p0.z - p.z);
      max_dis -= dis.norm();
      if (max_dis < 0) {
        max_index = i;
        break;
      }
    }

    // 在局部地图中搜索邻域的最近邻点, unused
    // octomap::point3d closest_pos;
    // if (!getClosestPoint(octomap::point3d(p.x, p.y, p.z), neigh_radius,
    //                      closest_pos)) {
    //   ROS_ERROR("<LocalPlanning::getClosestPoint>: No neighbour point.");
    //   return false;
    // }
    // printf(
    //     "<LocalPlanning::getClosestPoint>: [%.2f, %.2f, %.2f] -> [%.2f,
    //     %.2f,
    //     "
    //     "%.2f]\n",
    //     p.x, p.y, p.z, closest_pos.x(), closest_pos.y(), closest_pos.z());
    // path.poses[i].pose.position.x = closest_pos.x();
    // path.poses[i].pose.position.y = closest_pos.y();
    // path.poses[i].pose.position.z = closest_pos.z();
    // geometry_msgs::Quaternion q = path.poses[i].pose.orientation;
    // normals.push_back(Eigen::Vector3f(q.x, q.y, q.z));  // 取出法向量, unused
  }

  ros::WallDuration d = ros::WallTime::now() - t1;
  printf("<LocalPlanning::search>: search waypoint neighbor timecost %.3f s.\n",
         d.toSec());

  if (intepolate)
    for (int i = 0; i < path.poses.size(); i++) {  // 添加orientation
      float yaw, yaw1 = -INFINITY, yaw2 = -INFINITY;
      std::string index;
      if (i == 0) {  // 未给定初始方向则赋值
        yaw = GetOrientationBetweenTwoWaypoints(
            path.poses[i].pose.position, path.poses[i + 1].pose.position);
        index = "[-]";
      } else if (i == path.poses.size() - 1) {  // 未给定终点方向则赋值
        yaw = GetOrientationBetweenTwoWaypoints(path.poses[i - 1].pose.position,
                                                path.poses[i].pose.position);
        index = "[_]";
      } else {  // 中间的路点，则根据前后两点进行平滑
        yaw1 = GetOrientationBetweenTwoWaypoints(
            path.poses[i - 1].pose.position, path.poses[i].pose.position);
        yaw2 = GetOrientationBetweenTwoWaypoints(
            path.poses[i].pose.position, path.poses[i + 1].pose.position);
        index = "[|]";
        yaw = (yaw1 + yaw2) * 0.5;
      }

      Eigen::Quaternionf q(
          Eigen::AngleAxisf(yaw + 0.01, Eigen::Vector3f::UnitZ()));
      printf(
          "<LocalPlanning::getOrientation>: %s, yaw1=%.3f, yaw2=%.3f, "
          "yaw=%.3f, q=[%.3f,%.3f,%.3f,%.3f]\n",
          index.c_str(), yaw1 * 180 / M_PI, yaw2 * 180 / M_PI, yaw * 180 / M_PI,
          q.w(), q.x(), q.y(), q.z());

      path.poses[i].pose.orientation.w = q.w();
      path.poses[i].pose.orientation.x = q.x();
      path.poses[i].pose.orientation.y = q.y();
      path.poses[i].pose.orientation.z = q.z();
    }

  // 为路径上的每个点搜索构型
  std::vector<ConfigurationPoint> waypoints;
  float sum = 0;
  // max_index = 1;
  for (int i = 0; i < max_index; i++) {
    auto q = path.poses[i].pose.orientation;
    auto p = path.poses[i].pose.position;
    // Eigen::Vector4d wp(p.x, p.y, 0, 0);
    printf(
        "\n#########################################################\n"
        "<LocalPlanning::search>: PlanConfiguration for the %d-th waypoint\n",
        i);
    printf("<LocalPlanning::search>: goal_pos=[%.3f, %.3f, %.3f]\n", p.x, p.y,
           p.z);
    printf("<LocalPlanning::search>: goal_dir=[%.3f, %.3f, %.3f, %.3f]\n", q.w,
           q.x, q.y, q.z);

    Eigen::Quaternionf qq(q.w, q.x, q.y, q.z);
    qq.normalize();

    ConfigurationPoint way_point;
    way_point.position = Eigen::Vector3f(p.x, p.y, p.z);
    way_point.orientation = qq;

    ros::WallTime t = ros::WallTime::now();
    if (!PlanConfiguration(way_point)) {
      ROS_ERROR(
          "<LocalPlanning::search>: %d-th PlanConfiguration break, only %d "
          "configurations",
          i, static_cast<int>(waypoints.size()));
      if (1) {  // trick: 5
        way_point.flipper_angles = std::make_pair(-M_PI / 4, -M_PI / 4);
        way_point.offset = Eigen::Vector3f(0, 0, 0);
      } else {
        if (i == 0) return false;  // 第一个就失败了，则返回false
        break;
      }
    }
    way_point.timecost = ros::WallTime::now() - t;
    waypoints.push_back(way_point);
    if (debug_) usleep(1000 * 500);
  }

  std::cout << std::endl;
  for (int i = 0; i < waypoints.size(); i++) {
    auto wp = waypoints[i];
    printf("<LocalPlanning::search>: Waypoint[%02d]: %.3f, %.3f | %.3f, %.3f\n",
           i, wp.position.x(), wp.position.y(),
           wp.flipper_angles.first * 180.0 / M_PI,
           wp.flipper_angles.second * 180.0 / M_PI);
  }
  d = ros::WallTime::now() - t1;
  printf("<LocalPlanning::search>: local planning total timecost %.3f s.\n\n",
         d.toSec());
  PublishLocalPath(waypoints);  // , path
  if (wps_ptr != nullptr) *wps_ptr = waypoints;
  return true;
}

bool LocalPlanning::queryContactConfigurationServerFunction(
    path_planning::query_contact_configuration::Request& request,
    path_planning::query_contact_configuration::Response& response) {
  ros::WallTime t1 = ros::WallTime::now();

  if (!local_octree_ptr_) {
    ROS_ERROR(
        "<queryContactConfigurationServerFunction>: "
        "octree is null, return false");
    response.status = 0;  // no octree availiable
    return true;
  }

  geometry_msgs::Quaternion q = request.orientation;
  geometry_msgs::Point p = request.position;

  // Eigen::Vector4d wp(p.x, p.y, 0, 0);
  printf(
      "\n#########################################################\n"
      "<queryContactConfigurationServerFunction>: plan at "
      "p=(%.3f,%.3f,%.3f), q=(%.3f,%.3f,%.3f,%.3f)\n",
      p.x, p.y, p.z, q.x, q.y, q.z, q.w);
  ConfigurationPoint way_point;
  way_point.position = Eigen::Vector3f(p.x, p.y, p.z);
  way_point.orientation = Eigen::Quaternionf(q.w, q.x, q.y, q.z);

  if (!PlanConfiguration(way_point)) {
    response.status = 1;  // no feasible configuration
    return true;
  }

  response.front_flipper_angle.data = way_point.flipper_angles.first;
  response.rear_flipper_angle.data = way_point.flipper_angles.second;
  printf(
      "<queryContactConfigurationServerFunction>: "
      "Waypoint: %.3f, %.3f | %.3f, %.3f\n",
      way_point.position.x(), way_point.position.y(),
      way_point.flipper_angles.first * 180.0 / M_PI,
      way_point.flipper_angles.second * 180.0 / M_PI);

  ros::WallDuration d = ros::WallTime::now() - t1;
  printf(
      "<queryContactConfigurationServerFunction>: timecost %.3f "
      "ms.\n\n",
      d.toSec() * 1000);
  response.status = 2;  // feasible configuration
  return true;
}
