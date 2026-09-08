#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "mapping_module/world_representation/world_representation.h"

std::shared_ptr<octomap::IgTree> octree_ptr_;
ros::Publisher costmap_publisher_, elevationMap_publisher, heightDiff_publisher;
ros::Subscriber octomap_subscriber_;
ros::Subscriber bbox_subscriber_;

std::string map_frame_ = "map";                   // NOLINT
std::string footprint_frame_ = "base_footprint";  // NOLINT
std::string lidar_frame_ = "ouster_lidar";

const int MAP_SIZE = 200;
//高程过滤
const float LAYER_HEIGHT_MIN = -1.0;
const float LAYER_HEIGHT_MAX = 1.0;
const float SPARSITY_MIN = 0.7;
const float SLOPE_MIN = 0.6;

const int OBSTACLE_COST = 100;
const int FREE_COST = 0;
const int UNKNOWN_COST = -1;
//天然xy膨胀0.5m
const int NEIGH_SIZE = 1;
const float HEIGHT_DIFF_MAX = 20;
const float HEIGHT_DIFF_MIN = 0.7;//0.7

int width, height, offset_x, offset_y, offset_z;
float res;
bool map_flag = false;
bool first_map_flag = true;
tf::StampedTransform trans;
nav_msgs::OccupancyGrid costmap, elevationMap, heightDiffMap;
octomap::Boundingbox update_bbox;
geometry_msgs::Point current_position;

void OctomapCallback(const octomap_msgs::Octomap &msg) {
  octomap::IgTree temp_ig_tree(0.5);
  octomap::AbstractOcTree *aot = octomap_msgs::msgToMap(msg);
  if (aot) {
    if (!octree_ptr_) octree_ptr_.reset(new octomap::IgTree(0.2));
    octree_ptr_.reset(dynamic_cast<octomap::IgTree *>(aot));
    map_flag = true;
  }
}

void UpdateBBoxCallback(const visualization_msgs::Marker &msg) {
  if (msg.points.size() <= 1) {
    ROS_ERROR("<UpdateBBoxCallback>: Invalid bbox params !!!");
    return;
  }
  auto min_p = msg.points[0];
  auto max_p = msg.points[1];
  current_position = msg.points[2];
  current_position.x += trans.getOrigin().getX();
  current_position.y += trans.getOrigin().getY();
  current_position.z += trans.getOrigin().getZ();
  update_bbox.insertPoint(octomap::point3d(min_p.x, min_p.y, min_p.z));
  update_bbox.insertPoint(octomap::point3d(max_p.x, max_p.y, max_p.z));
}

void resetCostmapValuewithKey(octomap::OcTreeKey key) {
  auto p = octree_ptr_->keyToCoord(key);
  auto node = octree_ptr_->search(key);
  if (!node || !octree_ptr_->isNodeOccupied(node)) {  // unknown,free 则跳过
    return;
  }
  int xi = p.x() / res + offset_x;
  int yi = p.y() / res + offset_y;  // 加上偏移，避免为负
  if ((xi >= width || xi < 0) || (yi >= height || yi < 0)) {  // 限制范围
    return;
  }
  float delta_z = fabs(p.z() - current_position.z);
  // 判断机器人当前位置(传感器)高度，只对当前高度一定范围内的进行更新(reset)
  if (delta_z > LAYER_HEIGHT_MAX || delta_z < LAYER_HEIGHT_MIN) return;

  int idx = yi * width + xi;  // idx = y * width + x
  costmap.data[idx] = UNKNOWN_COST;
}

void checkHeightDiffWithKey(octomap::OcTreeKey key) {
  auto p = octree_ptr_->keyToCoord(key);
  auto node = octree_ptr_->search(key);
  if (!node || !octree_ptr_->isNodeOccupied(node)) {  // unknown,free 则跳过
    return;
  }

  int xr = p.x() / res + offset_x;
  int yr = p.y() / res + offset_y;  // 加上偏移，避免为负
  if ((xr >= width || xr < 0) || (yr >= height || yr < 0)) {  // 限制范围
    return;
  }

  float delta_z = fabs(p.z() - current_position.z);
  // 判断机器人当前位置(传感器)高度，只对当前高度一定范围内的进行更新
  if (delta_z > LAYER_HEIGHT_MAX || delta_z < LAYER_HEIGHT_MIN) return;

  int idx = yr * width + xr;  // idx = y * width + x
  // if (costmap.data[idx] == OBSTACLE_COST) {  // 已经是占据，则跳过
  //   return;
  // }

  int8_t max_height = elevationMap.data[idx];
  int8_t min_height = elevationMap.data[idx];
  int valid_cnt = 0;
  for (int i = -NEIGH_SIZE; i <= NEIGH_SIZE; i++) {
    for (int j = -NEIGH_SIZE; j <= NEIGH_SIZE; j++) {
      if (i == 0 && j == 0) continue;
      int xi = xr + i;
      int yi = yr + j;
      if (xi < 0 || xi >= width || yi < 0 || yi >= height) continue;
      int id = yi * width + xi;
      int8_t h = elevationMap.data[id];
      if (h == -128) continue;  // skip uninitialized cells
      max_height = std::max(h, max_height);
      min_height = std::min(h, min_height);
      valid_cnt++;
    }
  }
  // require at least 8 valid neighbors (out of 24) for meaningful height diff
  const int MIN_VALID_NEIGHBORS = 8;
  if (valid_cnt < MIN_VALID_NEIGHBORS) {
    heightDiffMap.data[idx] = UNKNOWN_COST;  // -1: insufficient data
    return;  // don't affect costmap
  }

  int8_t hi = std::min(std::max(abs(max_height - min_height), 0), 127);
  heightDiffMap.data[idx] = hi;

  float hf = hi * res;
  if (hf > HEIGHT_DIFF_MIN && hf < HEIGHT_DIFF_MAX) {
    costmap.data[idx] = OBSTACLE_COST;
  }
}

void setCostmapValueWithKey(octomap::OcTreeKey key) {
  auto p = octree_ptr_->keyToCoord(key);
  int xi = p.x() / res + offset_x;
  int yi = p.y() / res + offset_y;  // 加上偏移，避免为负
  if ((xi >= width || xi < 0) || (yi >= height || yi < 0)) {  // 限制范围
    return;
  }

  float delta_z = fabs(p.z() - current_position.z);
  // 判断机器人当前位置(传感器)高度，只对当前高度一定范围内的进行更新
  if (delta_z > LAYER_HEIGHT_MAX || delta_z < LAYER_HEIGHT_MIN) return;

  auto node = octree_ptr_->search(key);
  if (!node || !octree_ptr_->isNodeOccupied(node) ||
      node->getSparsity() > SPARSITY_MIN) {  // unknown 则跳过
    return;
  }

  int idx = yi * width + xi;                 // idx = y * width + x
  if (costmap.data[idx] == OBSTACLE_COST) {  // 已经是占据，则跳过
    return;
  }

  // ToDox: 增加step_height的判断
  costmap.data[idx] = node->getSlope() > SLOPE_MIN
                          ? OBSTACLE_COST
                          : FREE_COST;  // 100: occupied, 0: free
}

// 更新高程图
void setElevationMapValueWithKey(octomap::OcTreeKey key) {
  auto p = octree_ptr_->keyToCoord(key);
  int xi = p.x() / res + offset_x;
  int yi = p.y() / res + offset_y;  // 加上偏移，避免为负
  if ((xi >= width || xi < 0) || (yi >= height || yi < 0)) {  // 限制范围
    return;
  }
  auto node = octree_ptr_->search(key);
  if (!node || !octree_ptr_->isNodeOccupied(node)) {  // unknown 则跳过
    return;
  }

  float delta_z = fabs(p.z() - current_position.z);
  // 判断机器人当前位置(传感器)高度，只对当前高度一定范围内的进行更新
  if (delta_z > LAYER_HEIGHT_MAX || delta_z < LAYER_HEIGHT_MIN) return;

  int zi = std::min(std::max(static_cast<int>(p.z() / res), -128), 127);
  // printf("p.z = %.3f, zi = %d\n", p.z(), zi);
  int idx = yi * width + xi;  // idx = y * width + x
  if (zi > elevationMap.data[idx]) {
    elevationMap.data[idx] = (int8_t)zi;
  }
}

void initCostmap() {
  res = octree_ptr_->getResolution();
  width = MAP_SIZE / res;
  height = MAP_SIZE / res;
  offset_x = width / 2;
  offset_y = height / 2;
  costmap.info.origin.position.x = -offset_x * res;
  costmap.info.origin.position.y = -offset_y * res;
  costmap.info.width = width;
  costmap.info.height = height;
  costmap.info.map_load_time = ros::Time::now();
  costmap.info.resolution = res;
  costmap.data.clear();
  costmap.data.resize(height * width,
                      UNKNOWN_COST);  // -1:unknown, 0:free, 100:occupied
  costmap.header.stamp = ros::Time::now();
  costmap.header.frame_id = map_frame_;

  elevationMap = costmap;
  // elevationMap.data.resize(height * width, (int8_t)-128);  // valid >-128
  for (int i = 0; i < height * width; i++) {
    elevationMap.data[i] = -128;
  }

  heightDiffMap = costmap;  // init with UNKNOWN_COST(-1) instead of elevation values
  // -1 = unknown, 0 = flat, 1~127 = height diff in grid units
}

// 构建costmap地图
void updateCostmap() {
  // 如果为第一次，则更新全部地图
  ros::WallTime t1 = ros::WallTime::now();
  if (first_map_flag) {  //  ||
    initCostmap();
    first_map_flag = false;
    auto start = octree_ptr_->begin_leafs();
    auto end = octree_ptr_->end_leafs();
    for (auto iter = start; iter != end; iter++) {
      setCostmapValueWithKey(iter.getKey());
      setElevationMapValueWithKey(iter.getKey());
    }
    for (auto iter = start; iter != end; iter++) {
      checkHeightDiffWithKey(iter.getKey());
    }
    std::cout << "<generateCostmap>: update full costmap. ";
  } else {  // 如果不是第一次，则更新bbox中的
    costmap.header.stamp = ros::Time::now();
    elevationMap.header.stamp = ros::Time::now();
    update_bbox.enlarge(0.5);
    auto start = octree_ptr_->begin_leafs_bbx(update_bbox.minPoint(),
                                              update_bbox.maxPoint());
    auto end = octree_ptr_->end_leafs_bbx();

    for (auto iter = start; iter != end; iter++) {
      resetCostmapValuewithKey(iter.getKey());
    }
    for (auto iter = start; iter != end; iter++) {
      setCostmapValueWithKey(iter.getKey());
      setElevationMapValueWithKey(iter.getKey());
    }
    for (auto iter = start; iter != end; iter++) {
      checkHeightDiffWithKey(iter.getKey());
    }
    std::cout << "<generateCostmap>: update partial costmap. ";
  }
  // int cnt = 0;
  // for(int i=-128; i<128; i++) {
  //   elevationMap.data[cnt ++] = i;
  //   int val = elevationMap.data[i];
  //   printf("%d -> %d\n", i, val);
  // }
  ros::WallTime t2 = ros::WallTime::now();
  std::cout << "timecost " << (t2 - t1).toSec() << "s" << std::endl;
  costmap_publisher_.publish(costmap);
  elevationMap_publisher.publish(elevationMap);
  heightDiff_publisher.publish(heightDiffMap);
  update_bbox.reset();
}

bool getSensorToFootprintTF(tf::StampedTransform *trans,
                            ros::Time timestamp = ros::Time(0)) {
  static tf::TransformListener listener;
  try {
    listener.waitForTransform(footprint_frame_, lidar_frame_, timestamp,
                              ros::Duration(0));
    listener.lookupTransform(footprint_frame_, lidar_frame_, timestamp, *trans);
  } catch (...) {
    ROS_ERROR("Listen TF [%.3f] (%s -> %s) timeout!", timestamp.toSec(),
              footprint_frame_.c_str(), lidar_frame_.c_str());
    return false;
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_costmaper_node");

  ros::NodeHandle nh;
  octomap_subscriber_ =
      nh.subscribe("mapping_module/global_octree", 1, &OctomapCallback);
  bbox_subscriber_ = nh.subscribe("mapping_module/global_update_bbox_param", 1,
                                  &UpdateBBoxCallback);
  costmap_publisher_ =
      nh.advertise<nav_msgs::OccupancyGrid>("mapping_module/global_costmap", 1);
  elevationMap_publisher = nh.advertise<nav_msgs::OccupancyGrid>(
      "mapping_module/global_elevation_map", 1);
  heightDiff_publisher = nh.advertise<nav_msgs::OccupancyGrid>(
      "mapping_module/global_height_diff_map", 1);
  nh.param<std::string>("global_costmaper_node/world_frame", map_frame_, "map");
  nh.param<std::string>("global_costmaper_node/footprint_frame",
                        footprint_frame_, "base_footprint");
  nh.param<std::string>("global_costmaper_node/lidar_frame", lidar_frame_,
                        "ouster_lidar");

  while (!getSensorToFootprintTF(&trans) && ros::ok()) {
    sleep(1);
  }

  ros::Rate rate(10);
  while (ros::ok()) {
    if (map_flag && !update_bbox.isReset()) {  // 没收到bbox或地图
      map_flag = false;
      updateCostmap();
    }
    rate.sleep();
    ros::spinOnce();
  }
}
