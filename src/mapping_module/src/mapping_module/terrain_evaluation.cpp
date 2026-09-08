/* Copyright Year: 2026
 * Copyright Owner: Networked Robotics and Systems Lab
 * Authors: Yuxiang Li, Kun Chen, Haoyao Chen
 */
 
#include "mapping_module/terrain_evaluation.h"

#include <geometry_msgs/Point.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <unordered_set>
namespace mapping_module {

TerrainEvaluation::TerrainEvaluation(std::shared_ptr<WorldRepresentation> wr) {
  ros::NodeHandle nh("/mapping_module");
  nh.getParam("map_frame", map_frame_);
  nh.getParam("filling_radius", filling_radius_);
  nh.getParam("frame_height", frame_height_);

  ros::NodeHandle nh_local("/mapping_module/terrain_evaluation/");
  nh_local.getParam("roughness_threshold", roughness_threshold_);
  nh_local.getParam("sparsity_threshold", sparsity_threshold_);
  nh_local.getParam("slope_threshold", slope_threshold_);
  nh_local.getParam("traversability_threshold", traversability_threshold_);
  nh_local.getParam("costmap_enable", costmap_enable_);
  nh_local.getParam("costmap_height", costmap_height_);
  nh_local.getParam("costmap_size", costmap_size_);
  nh_local.getParam("collision_neighbor", collision_neighbor_);
  std::cout << "<TerrainEvaluation>: slope_threshold_=" << slope_threshold_
            << ", sparsity_threshold_=" << sparsity_threshold_
            << ", traversability_threshold_=" << traversability_threshold_
            << std::endl;

  nh_local.getParam("roughness_weight", roughness_weight_);
  nh_local.getParam("sparsity_weight", sparsity_weight_);
  nh_local.getParam("slope_weight", slope_weight_);
  nh_local.getParam("traversability_weight", traversability_weight_);
  nh_local.getParam("neighbor_radius", neighbor_radius_);
  nh_local.getParam("points_number_threshold", points_number_threshold_);

  local_drivable_pcd_publisher_ = nh_local.advertise<sensor_msgs::PointCloud2>(
      "local_drivable_pointcloud", 1);

  if (costmap_enable_) {
    costmap_publisher_ =
        nh_local.advertise<nav_msgs::OccupancyGrid>("costmap", 1);
    elevation_map_publisher_ =
        nh_local.advertise<nav_msgs::OccupancyGrid>("elevation_map", 1);
    traversibility_map_publisher_ =
        nh_local.advertise<nav_msgs::OccupancyGrid>("traversibility_map", 1);
  }

  resetTerrainRepresentation(wr);

  query_nearest_drivable_point_server_ = nh_local.advertiseService(
      "query_nearest_drivable_point",
      &TerrainEvaluation::queryDrivablePointServerCallback, this);
  // query_all_drivable_pcd_server_ = nh_local.advertiseService(
  //     "query_all_drivable_pcd",
  //     &TerrainEvaluation::queryAllDrivablePCDServerCallback, this);
  // query_terrain_attribute_server_ = nh_local.advertiseService(
  //     "query_terrain_attribute",
  //     &TerrainEvaluation::queryTerrainAttributeServerCallback, this);

  terrain_thread_ =
      std::thread(std::bind(&TerrainEvaluation::terrainUpdate, this));
}

TerrainEvaluation::~TerrainEvaluation() {
  printf("<~TerrainEvaluation>: set cancel\n");
  setCancel();
  // query_nearest_drivable_point_server_.shutdown();
  // query_all_drivable_pcd_server_.shutdown();
  terrain_thread_.join();
  printf("<~TerrainEvaluation>: thread exit\n");
}

void TerrainEvaluation::resetTerrainRepresentation(
    std::shared_ptr<WorldRepresentation> wr) {
  world_representation_ = wr;
  octree_ptr_ = world_representation_->getGlobalOcTreeSharedPtr();

  // reset DisjointSet and Kdtree
  std::lock_guard<std::mutex> lock(kdtree_mutex_);
  drivable_disjoint_root_valid_ = false;
  kdtree_initialized_ = false;
  drivable_disjoint_set_.reset(
      new DisjointSet<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash>);
  kdtree_ptr_.reset(new KD_TREE<PointType>(0.3, 0.6, 0.2));
}

void TerrainEvaluation::terrainInit(Eigen::Isometry3d &current_pose) {
  std::cout << "<terrainInit>: init position:"
            << current_pose.translation().transpose() << std::endl;
  // 根据lidar_frame高度，生成邻域高密度的点云，然后插入
  float res = world_representation_->getGlobalOctreePtr()->getResolution();
  if (filling_radius_ < res) {
    ROS_ERROR("<terrainInit>: filling_radius(%.3f) < res(%.3f)\n",
              filling_radius_, res);
    return;
  }

  float z = current_pose.translation().z() + frame_height_;
  if (fabs(current_pose.translation().x()) > 0.3 ||
      fabs(current_pose.translation().y()) > 0.3 || fabs(z) > 0.3) {
    ROS_ERROR(
        "<terrainInit>: dis=(%.3f,%.3f,%.3f), far away from origin. jump "
        "filling terrain\n",
        fabs(current_pose.translation().x()),
        fabs(current_pose.translation().y()), fabs(z));

    return;  // 不在原点就不填充
  }

  // 每个格子里要填充10点以上
  int times = 10;  // 0.2/10 = 0.02
  octomap::KeySet update_all_voxels;
  octomap::point3d update_center(0, 0, 0);
  int ri = filling_radius_ / res * times + 1;
  for (int i = -ri; i < ri; i++) {
    for (int j = -ri; j < ri; j++) {
      float x = i * res / times;
      float y = j * res / times;

      octomap::point3d p3d(x, y, z);
      // 先更新全局
      auto key = world_representation_->getGlobalOctreePtr()->coordToKey(p3d);
      world_representation_->getGlobalOctreePtr()->updateNode(key, true);
      auto node = world_representation_->getGlobalOctreePtr()->search(key);
      node->insertPoint(p3d);
      if (update_all_voxels.count(key) == 0) {  // 更新的全局地图栅格去重
        update_all_voxels.insert(key);
      }
      // 再更新局部地图
      key = world_representation_->getLocalOctreePtr()->coordToKey(p3d);
      world_representation_->getLocalOctreePtr()->updateNode(key, true);
    }
  }

  updateMapTerrainAttribute(update_all_voxels, &update_center);
}

// enable_terrain_ 打开时才会开启线程执行该函数
void TerrainEvaluation::terrainUpdate() {
  printf("<terrainUpdate>: thread started\n");
  Eigen::Isometry3d current_pose;
  if (world_representation_->getSensorPoseEigen(&current_pose)) {
    std::lock_guard<std::mutex> lock_octree(
        world_representation_->globalOctomapMutex());
    terrainInit(current_pose);
  } else {
    ROS_ERROR(
        "<TerrainEvaluation::terrainUpdate>: get tf failed. Did not fill "
        "original "
        "terrain.");
  }
  ros::WallTime t1 = ros::WallTime::now();
  while (ros::ok() && !checkCancel()) {
    int list_size = 0;
    {  // 获取list长度
      std::lock_guard<std::mutex> lock(
          world_representation_->globalUpdateVoxelsListMutex());
      list_size = world_representation_->globalUpdateVoxelsList().size();
    }
    if (list_size == 0) {  // 长度为0则继续等待
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    ros::WallDuration d = ros::WallTime::now() - t1;
    octomap::KeySet update_all_voxels;
    octomap::point3d update_center(0, 0, 0);
    int cnt = 0;
    if (list_size >= 3 || d.toSec() > 0.5) {  // list太长、或超时
      t1 = ros::WallTime::now();
      {  // 从List中pop出front
        std::lock_guard<std::mutex> lock(
            world_representation_->globalUpdateVoxelsListMutex());
        while (world_representation_->globalUpdateVoxelsList().size() > 0) {
          std::pair<octomap::KeySet, std::vector<octomap::IgTreeNode *>>
              update_voxels =
                  world_representation_->globalUpdateVoxelsList().front();
          update_center +=
              world_representation_->globalUpdatePositionList().front();
          if (update_voxels.first.size() != update_voxels.second.size())
            continue;
          for (octomap::KeySet::iterator it = update_voxels.first.begin();
               it != update_voxels.first.end(); ++it) {
            if (update_all_voxels.count(*it) <= 0) {  // 如果没有则插入
              update_all_voxels.insert(*it);
            }
          }
          world_representation_->globalUpdateVoxelsList().pop_front();
          world_representation_->globalUpdatePositionList().pop_front();
          cnt++;
        }
      }
      if (cnt != 0) update_center /= cnt;

      // 只对占据栅格邻域更新，相比更新bbox所有栅格速度更快
      std::lock_guard<std::mutex> lock_octree(
          world_representation_->globalOctomapMutex());
      updateMapTerrainAttribute(update_all_voxels, &update_center);
    } else {  // 长度太短、或时间间隔不够长
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  printf("<TerrainEvaluation::terrainUpdate>: thread exit\n");
}

bool getUpperVoxels(const octomap::IgTree &tree,
                    std::vector<octomap::IgTreeNode *> &nodes_in,   // NOLINT
                    std::vector<octomap::OcTreeKey> &keys_in,       // NOLINT
                    std::vector<octomap::IgTreeNode *> &nodes_out,  // NOLINT
                    std::vector<octomap::OcTreeKey> &keys_out,      // NOLINT
                    std::vector<octomap::IgTreeNode *> &nodes_occ,  // NOLINT
                    std::vector<octomap::OcTreeKey> &keys_occ) {    // NOLINT
  std::unordered_map<octomap::OcTreeKey, octomap::key_type,
                     octomap::OcTreeKey::KeyHash>
      elevation_keys;  // [key]=z
  std::unordered_map<octomap::OcTreeKey, octomap::IgTreeNode *,
                     octomap::OcTreeKey::KeyHash>
      elevation_nodes;  // [key]=node

  keys_occ.clear();
  nodes_occ.clear();
  for (int i = 0; i < keys_in.size(); i++) {
    if (!nodes_in[i] || !tree.isNodeOccupied(nodes_in[i])) continue;
    // if (i > 8 && elevation_keys.size() <= 2) return false;
    if (i > 26 && elevation_keys.size() == 0) return false;  // 离群点
    keys_occ.push_back(keys_in[i]);
    nodes_occ.push_back(nodes_in[i]);
    auto key = keys_in[i];
    octomap::key_type z = key.k[3];
    key.k[2] = 0;                     // 转成2D
    if (elevation_keys.count(key)) {  // 存在
      if (z > elevation_keys[key]) {
        elevation_keys[key] = z;
        elevation_nodes[key] = nodes_in[i];
      }
    } else {
      elevation_keys[key] = z;
      elevation_nodes[key] = nodes_in[i];
    }
  }

  nodes_out.clear();
  keys_out.clear();
  for (auto p : elevation_keys) {
    auto key = p.first;
    key.k[2] = p.second;
    keys_out.push_back(key);
    nodes_out.push_back(elevation_nodes[p.first]);
  }
  return true;
}

void TerrainEvaluation::updateVoxelTerrainAttribute(
    octomap::OcTreeKey &iter_key, octomap::IgTreeNode *iter_node,
    octomap::KeySet *update_voxels, octomap::point3d *update_position,
    octomap::KeySet *neigh_voxels) {
  // 对新bbox中的栅格，更新粗糙度、密度、斜率，以及可通行性
  if (!iter_node || !octree_ptr_->isNodeOccupied(iter_node))  // 只处理占据的
    return;  // iter_node->pointCount() == 0  // BUG:
             // 刚观测到的占据栅格，其中点数为0

  // 搜索邻域栅格
  std::vector<octomap::OcTreeKey> neigh_keys, neigh_keys_all, neigh_keys_occ;
  std::vector<octomap::IgTreeNode *> neigh_nodes, neigh_nodes_all,
      neigh_nodes_occ;
  octomap::getNeighbor125(*octree_ptr_, iter_key, &neigh_nodes_all,
                          &neigh_keys_all);
  if (!getUpperVoxels(*octree_ptr_, neigh_nodes_all, neigh_keys_all,
                      neigh_nodes, neigh_keys, neigh_nodes_occ,
                      neigh_keys_occ)) {  // 提取top层，用于地形计算
    octree_ptr_->updateNode(iter_key, false);  // 离群点滤掉
    return;
  }
  if (neigh_voxels && update_voxels) {  // 传入了neigh_voxels和update_voxels
    for (int i = 0; i < neigh_nodes_occ.size(); i++) {
      auto key = neigh_keys_occ[i];
      auto node = neigh_nodes_occ[i];
      if (neigh_voxels->find(key) == neigh_voxels->end() &&
          update_voxels->find(key) == update_voxels->end()) {
        neigh_voxels->insert(key);
      }
    }
  }

  // 用于融合NDT协方差
  Eigen::Matrix3f sigma_fused = iter_node->cov();
  Eigen::Vector3f mu_fused = iter_node->mu();
  int pt_cnt_fused = iter_node->pointCount() == 0 ? 1 : iter_node->pointCount();
  // 用于计算均值协方差
  Eigen::Matrix3f sigma_avg = Eigen::Matrix3f::Zero();
  Eigen::Vector3f mu_avg = Eigen::Vector3f::Zero();
  int pt_cnt_avg = 0;
  octomap::point3d voxel_center(0, 0, 0);
  Eigen::Vector3f center = mu_fused;
  int occ_count = 0;

  std::vector<Eigen::Vector3f> occ_points;
  std::vector<octomap::IgTreeNode *> occ_nodes;
  Eigen::MatrixXf A(neigh_nodes.size(), 3);
  float max_height = -1e3;
  float min_height = 1e3;
  for (int k = 0; k < neigh_nodes.size(); k++) {  // 邻域内计算均值、协方差
    // if (!neigh_nodes[k] || !octree_ptr_->isNodeOccupied(neigh_nodes[k]))
    //   continue;
    // 除了不存在、非占据的，不能忽略掉其他任何栅格！！！
    octomap::fuseTwoDistributions(neigh_nodes[k], mu_fused, sigma_fused,
                                  pt_cnt_fused);  // 融合NDT
    auto pt = neigh_nodes[k]->mu();
    // 用于计算均值协方差
    octomap::point3d voxel_pt = octree_ptr_->keyToCoord(neigh_keys[k]);
    voxel_center += voxel_pt;
    A.row(pt_cnt_avg++) =
        Eigen::Vector3f(voxel_pt.x(), voxel_pt.y(), voxel_pt.z());
    // float factor = 1 / static_cast<float>(pt_cnt_avg + 1);
    // sigma_avg = pt_cnt_avg * factor * sigma_avg +
    //             pt_cnt_avg * factor * factor * (voxel - mu_avg) *
    //                 (voxel - mu_avg).transpose();
    // mu_avg = mu_avg + factor * (voxel - mu_avg);
    max_height = std::max(max_height, pt.z());
    min_height = std::min(min_height, pt.z());

    occ_points.push_back(pt);
    occ_nodes.push_back(neigh_nodes[k]);

    center += pt;
    occ_count++;
  }

  center /= occ_count;
  voxel_center /= pt_cnt_avg;

  Eigen::Vector3f normal_vector(0, 0, 0);
  octomap::EigenInfo ei;
  // 点数比较少时（少于栅格数），用栅格中心计算法向量
  if (pt_cnt_fused <= pt_cnt_avg) {
    if (pt_cnt_avg > 4) {  // 栅格数量太少，栅格中心计算也不准
      Eigen::MatrixXf A_ = A.block(0, 0, pt_cnt_avg, 3);
      for (int i = 0; i < A_.rows(); i++)
        A_.row(i) -= Eigen::Vector3f(voxel_center.x(), voxel_center.y(),
                                     voxel_center.z());
      Eigen::MatrixXf ATA = A_.transpose() * A_;
      Eigen::JacobiSVD<Eigen::MatrixXf> svd(ATA, Eigen::ComputeFullV);
      Eigen::Vector3f n = svd.matrixV().col(2);  // descreasing order
      ei.min_eigen_val = svd.singularValues()(2);
      ei.mid_eigen_val = svd.singularValues()(1);
      ei.max_eigen_val = svd.singularValues()(0);
      ei.min_eigen_vec = svd.matrixV().col(2);
      ei.mid_eigen_vec = svd.matrixV().col(1);
      ei.max_eigen_vec = svd.matrixV().col(0);
      iter_node->setFuseEigen(ei);
    } else if (pt_cnt_avg < 2) {
      octree_ptr_->updateNode(iter_key, false);
      return;
    }
    // 数量太少则不计算，其法向量为(0,0,0)
  } else {  // 计算法向量
    iter_node->setFuseMu(mu_fused);
    iter_node->setFuseSigma(sigma_fused);
    iter_node->setFuseCount(pt_cnt_fused);
    iter_node->setFuseEigen();
    ei = iter_node->getFuseEigen();
  }

  if (ei.min_eigen_vec.norm() == 0) {
    octree_ptr_->updateNode(iter_key, false);
    return;
  }

  normal_vector = ei.min_eigen_vec / ei.min_eigen_vec.norm();
  // 根据法向量计算斜率：法向量与z(0,0,1)夹角
  float angle = acos(fabs(normal_vector.dot(Eigen::Vector3f::UnitZ())));
  iter_node->setSlope(angle);
  iter_node->setHeightDiff(max_height - min_height);

  // 1.1 计算旋转矩阵：法向量至z(0,0,1)
  Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(
      normal_vector, Eigen::Vector3f::UnitZ());

  // 1.1). 旋转点云，使法向量旋转至z(0,0,1)
  for (auto &p : occ_points) {
    p = rotation * (p - iter_node->mu());
  }

  // 1.2). 2D投影，选择最高点
  std::unordered_map<octomap::OcTreeKey, std::pair<float, int>,
                     octomap::OcTreeKey::KeyHash>
      height_map;  // [key]: (height, index)
  for (int i = 0; i < occ_points.size(); i++) {
    auto p = occ_points.at(i);
    auto key = octree_ptr_->coordToKey(
        octomap::point3d(p.x(), p.y(), 0));          // 用2D key去重
    if (height_map.find(key) == height_map.end()) {  // 不存在时：
      height_map[key] = std::make_pair(p.z(), i);    // 记录高度及索引
    } else {                                         // 存在时：
      if (height_map[key].first < p.z()) {           // 更新高度及索引
        height_map[key].first = p.z();
        height_map[key].second = i;
      }
    }
  }

  // 1.3). 累计投影后的栅格数、栅格内点数
  int points_projected = 0, voxels_projected = 0;
  height_map.size();
  for (auto p : height_map) {
    int cnt = std::min(occ_nodes.at(p.second.second)->pointCount(),
                       points_number_threshold_);  // 低密度点数阈值20
    points_projected += cnt;
    voxels_projected++;
  }

  // 1.4). 计算投影后的点数/栅格比
  float density_projected =  // 归一化，栅格中点数超过阈值则不再更新
      voxels_projected == 0 ? 0
                            : points_projected * 1.0 /
                                  (voxels_projected * points_number_threshold_);
  iter_node->setSparsity(1 - density_projected);

  // 2. 根据协方差计算粗糙度
  float sum = ei.min_eigen_val + ei.max_eigen_val + ei.mid_eigen_val;
  // p1 > p2 > p3
  // float P1 = ei.max_eigen_val / sum;
  float P2 = ei.mid_eigen_val / sum;
  float P3 = ei.min_eigen_val / sum;
  // float roughness = ei.min_eigen_val / ei.max_eigen_val;
  float roughness = 1 - (P2 - P3) / (P2 + P3);
  iter_node->setRoughness(roughness);
  float traverse_cost =
      traversability_weight_ * (sparsity_weight_ * iter_node->getSparsity() +
                                slope_weight_ * iter_node->getSlope() +
                                roughness_weight_ * iter_node->getRoughness());
  iter_node->setTraversability(traverse_cost);
  if (iter_node->getSlope() == 0 && iter_node->getSparsity() == 1) {
    octree_ptr_->updateNode(iter_key, false);
  }
}

// 更新包围框内的体素, unused due to low efficiency
void TerrainEvaluation::updateMapTerrainAttribute(
    octomap::point3d update_position, octomap::Boundingbox update_bbox) {
  std::cout << "<updateTerrainAttribute>: original update_bbox = "
            << update_bbox << std::endl;
  update_bbox.enlarge(neighbor_radius_);  // 对input_pc_bbox_扩大r邻域
  std::cout << "<updateTerrainAttribute>: enlarged update_bbox = "
            << update_bbox << std::endl;

  octomap::IgTree::leaf_bbx_iterator iter_begin = octree_ptr_->begin_leafs_bbx(
      update_bbox.minPoint(), update_bbox.maxPoint());
  octomap::IgTree::leaf_bbx_iterator iter_end = octree_ptr_->end_leafs_bbx();
  for (auto it = iter_begin; it != iter_end; ++it) {  // 提取包围框中的格子
    // 对新bbox中的栅格，更新粗糙度、密度、斜率，以及可通行性
    octomap::OcTreeKey iter_key = it.getKey();
    octomap::IgTreeNode *iter_node = octree_ptr_->search(iter_key);
    updateVoxelTerrainAttribute(iter_key, iter_node, nullptr, &update_position);
  }
}

// 更新给定体素集合，入口!!!
void TerrainEvaluation::updateMapTerrainAttribute(
    octomap::KeySet &update_voxels, octomap::point3d *update_position) {
  ros::WallTime t1 = ros::WallTime::now();
  int update_voxels_num = update_voxels.size();
  if (update_voxels_num <= 0) {
    ROS_ERROR("<updateMapTerrainAttribute>: no update_voxels !!!");
    return;
  }
  octomap::KeySet neigh_voxels;

  // ROS_WARN("<updateMapTerrainAttribute>: leaf num = %ld",
  //          octree_ptr_->getNumLeafNodes());
  // 先对当前帧的占据格子(update_voxels)，更新地形属性、提取邻居占据栅格
  for (octomap::KeySet::iterator it = update_voxels.begin();
       it != update_voxels.end(); ++it) {
    octomap::OcTreeKey iter_key = *it;
    octomap::IgTreeNode *iter_node = octree_ptr_->search(iter_key);
    updateVoxelTerrainAttribute(iter_key, iter_node, &update_voxels,
                                update_position, &neigh_voxels);
  }

  // 对当前帧占据栅格的邻居栅格(neigh_voxels)，更新地形属性
  for (octomap::KeySet::iterator it = neigh_voxels.begin();
       it != neigh_voxels.end(); ++it) {
    octomap::OcTreeKey iter_key = *it;
    octomap::IgTreeNode *iter_node = octree_ptr_->search(iter_key);
    updateVoxelTerrainAttribute(iter_key, iter_node, (octomap::KeySet *)nullptr,
                                update_position, (octomap::KeySet *)nullptr);
    update_voxels.insert(iter_key);  // 将neigh_voxels合并到update_voxels
  }

  // 根据最新的地形属性，更新栅格的碰撞，保存新出现的无碰撞栅格
  std::vector<octomap::OcTreeKey> new_drivable_set;
  std::vector<octomap::IgTreeNode *> new_drivable_nodes;
  std::vector<octomap::OcTreeKey> new_indrivable_set;
  std::vector<octomap::IgTreeNode *> new_indrivable_nodes;
  for (octomap::KeySet::iterator it = update_voxels.begin();
       it != update_voxels.end(); ++it) {
    octomap::OcTreeKey iter_key = *it;
    octomap::IgTreeNode *iter_node = octree_ptr_->search(iter_key);
    int ret = updateVoxelTerrainCollision(iter_key, iter_node);
    if (ret == 1) {
      new_drivable_set.push_back(iter_key);
      new_drivable_nodes.push_back(iter_node);
    } else if (ret == -1) {
      new_indrivable_set.push_back(iter_key);
      new_indrivable_nodes.push_back(iter_node);
    }
  }

  // 将无碰撞栅格更新到disjoint set
  updateDrivableDisjointSet(new_drivable_set, new_drivable_nodes,
                            new_indrivable_set, new_indrivable_nodes,
                            *update_position);
  printf(
      "<updateMapTerrainAttribute>: update+neigh: %d+%d=%d voxels, center: "
      "[%.1f, %.1f, %.1f], timecost = %.3fs\n",
      update_voxels_num, static_cast<int>(neigh_voxels.size()),
      static_cast<int>(update_voxels.size()), update_position->x(),
      update_position->y(), update_position->z(),
      (ros::WallTime::now() - t1).toSec());
}

// 满足约束返回true
bool TerrainEvaluation::checkNodeConstraints(octomap::IgTreeNode *node,
                                             bool debug_msg) {
  if (debug_msg) {
    printf(
        "<checkNodeConstraints>: slope:%.3f/%.3f(<), densty:%.3f/%.3f(<), "
        "roughness:%.3f/%.3f(<), traverseCost:%.3f/%.3f(<)\n",
        node->getSlope(), slope_threshold_, node->getSparsity(),
        sparsity_threshold_, node->getRoughness(), roughness_threshold_,
        node->getTraversability(), traversability_threshold_);
  }
  if (node->getSlope() > slope_threshold_ ||
      node->getSparsity() > sparsity_threshold_ ||
      node->getRoughness() > roughness_threshold_ ||
      node->getTraversability() > traversability_threshold_)  // 硬约束
    return false;
  return true;
}

// 无碰撞返回true
bool TerrainEvaluation::checkNeighborCells(octomap::OcTreeKey key,
                                           bool debug_msg) {
  bool flag[64];
  memset(flag, 0, sizeof(flag));
  static int index_x[] = {
      0, 0, 0, 1,  -1, 1, 1,  -1, -1,  // 9: 2s
      0, 0, 2, -2, 2,  2, -2, -2,      // 17: 3s
      0, 0, 3, -3, 3,  3, -3, -3       // 25: 4.6s
  };
  static int index_y[] = {
      0, 1,  -1, 0, 0, 1,  -1, 1,  -1,  // (0-8)9
      2, -2, 0,  0, 2, -2, 2,  -2,      // +8, 17
      3, -3, 0,  0, 3, -3, 3,  -3       // +8, 25
  };

  bool stop = false;
  pcl::PointCloud<pcl::PointXYZI> neigh_points;
  for (int z = 3; z >= -3; z--) {
    for (int i = 0; i < collision_neighbor_; i++) {
      if (flag[i]) continue;  // 该位置向下投射有了占据点则跳过
      octomap::OcTreeKey tmp = key;
      tmp.k[0] += index_x[i];
      tmp.k[1] += index_y[i];
      tmp.k[2] += z;

      pcl::PointXYZI pc;
      auto p3d = octree_ptr_->keyToCoord(tmp);
      pc.x = p3d.x();
      pc.y = p3d.y();
      pc.z = p3d.z();
      pc.intensity = -4;  // 正常为紫色，其他为高度渐变颜色
      octomap::IgTreeNode *node = octree_ptr_->search(tmp);
      if (node && octree_ptr_->isNodeOccupied(node)) {
        flag[i] = true;
        if (z == 3) {  // 第3层不能有占据
          pc.intensity = z;
          stop = true;
        } else if (!checkNodeConstraints(node)) {  // 其它层的占据必须为可通行
          pc.intensity = 4;                        // 不满足约束为红色
          stop = true;
        }
      } else {                              // free or unknown
        if (z == -2 && 1 <= i && i <= 8) {  // 1邻域，-2层不能有free
          pc.intensity = z;
          stop = true;
        }
        if (z == -3 && 8 <= i) {  // 2邻域，-3层不能有free
          pc.intensity = z;
          stop = true;
        }
      }
      neigh_points.push_back(pc);
      if (stop) break;
    }
    if (stop) break;
  }
  if (debug_msg) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(neigh_points, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time::now();
    local_drivable_pcd_publisher_.publish(msg);
  }
  return !stop;
}

bool TerrainEvaluation::checkDrivable(octomap::OcTreeKey key) {
  if (!drivable_disjoint_root_valid_) return false;
  if (!drivable_disjoint_set_->count(key) ||
      drivable_disjoint_set_->find(key) != drivable_disjoint_root_) {
    return false;
  }
  return true;
}

void TerrainEvaluation::updateDrivableDisjointSet(
    std::vector<octomap::OcTreeKey> &update_voxels,
    std::vector<octomap::IgTreeNode *> &update_nodes,
    std::vector<octomap::OcTreeKey> &revoke_voxels,
    std::vector<octomap::IgTreeNode *> &revoke_nodes,
    octomap::point3d update_position) {
  std::lock_guard<std::mutex> lock(kdtree_mutex_);
  // 找到最近邻的key作为root
  if (!drivable_disjoint_root_valid_) {
    float min_dis = 1e9;
    for (auto iter_key : update_voxels) {
      octomap::point3d dis =
          octree_ptr_->keyToCoord(iter_key) - update_position;
      if (dis.norm() < min_dis) {
        min_dis = dis.norm();
        drivable_disjoint_root_ = iter_key;
        drivable_disjoint_root_valid_ = true;
      }
    }
  }

  // 先插入disjoint set
  for (auto iter_key : update_voxels) {
    if (!drivable_disjoint_set_->count(iter_key))
      drivable_disjoint_set_->insert(iter_key);
  }

  // 再与邻域合并
  for (auto iter_key : update_voxels) {
    std::vector<octomap::OcTreeKey> neigh_keys;
    octomap::getNeighborSix(*octree_ptr_, iter_key,
                            (std::vector<octomap::IgTreeNode *> *)nullptr,
                            &neigh_keys);
    // octomap::getNeighbor27(*octree_ptr_, iter_key,
    //                         (std::vector<octomap::IgTreeNode *> *)nullptr,
    //                         &neigh_keys);
    for (auto &key : neigh_keys) {
      if (drivable_disjoint_set_->count(key))
        drivable_disjoint_set_->merge(iter_key, key);
    }
  }

  // 更新root
  if (drivable_disjoint_root_valid_)
    drivable_disjoint_root_ =
        drivable_disjoint_set_->find(drivable_disjoint_root_);

  // 再遍历一遍，将连通的体素，setConnected
  // 通过连通条件筛选可通行体素，避免房顶等孤立的可通行区域
  PointVector add_points, del_points;
  int iter_key_cnt = 0;
  for (auto iter_key : update_voxels) {
    if (drivable_disjoint_set_->find(iter_key) == drivable_disjoint_root_) {
      update_nodes.at(iter_key_cnt)->setConnected(true);  // 可通行
      auto p = octree_ptr_->keyToCoord(iter_key);
      add_points.push_back(PointType(p.x(), p.y(), p.z()));
    }
    iter_key_cnt++;
  }
  iter_key_cnt = 0;
  for (auto iter_key : revoke_voxels) {
    revoke_nodes.at(iter_key_cnt)->setConnected(false);  // 不可通行
    auto p = octree_ptr_->keyToCoord(iter_key);
    del_points.push_back(PointType(p.x(), p.y(), p.z()));
    iter_key_cnt++;
  }

  // 更新ikdtree
  if (!kdtree_initialized_) {
    if (add_points.size() > 0) {
      kdtree_ptr_->Build(add_points);  // 第一次必须build，后面才能add
      kdtree_initialized_ = true;
    }
  } else if (add_points.size() > 0) {
    kdtree_ptr_->Add_Points(add_points, true);
  }

  if (del_points.size() > 0) {
    kdtree_ptr_->Delete_Points(del_points);
  }

  if (costmap_enable_) {
    geometry_msgs::Point p;
    p.x = update_position.x();
    p.y = update_position.y();
    p.z = update_position.z();
    generateCostmap(p);
  }
}

// 1: add, -1: delete, 0: dont care
int TerrainEvaluation::updateVoxelTerrainCollision(
    octomap::OcTreeKey iter_key, octomap::IgTreeNode *iter_node) {
  if (!iter_node || !octree_ptr_->isNodeOccupied(iter_node))
    return 0;  // unknown, free的直接跳过

  // bool old_val = iter_node->getCollision();
  if (checkNodeConstraints(iter_node) && checkNeighborCells(iter_key)) {
    iter_node->setCollision(false);  // 无碰撞，绿色
  } else {
    iter_node->setCollision(true);  // 碰撞，红色
  }

  // bool new_val = iter_node->getCollision();
  // if (old_val != new_val)
  // {  // 发生变化的，加上会导致部分不更新!!!!!
  //   if (new_val == 0.5) {  // 由未知/不可通行，变为可通行
  //     return 1;
  //   }
  //   // if (new_val == 1 && old_val == 0.5) {  // 由可通行，变为不可通行
  //   if (new_val == 1) {  // 由未知/可通行，变为不可通行
  //     return -1;
  //   }
  // }

  return iter_node->getCollision() ? -1 : 1;
}

// unused. for exploration
bool TerrainEvaluation::queryAllDrivablePCDServerCallback(
    query_all_drivable_pcd::Request &request,  // NOLINT
    query_all_drivable_pcd::Response &response) {
  std::lock_guard<std::mutex> lock(kdtree_mutex_);
  ros::WallTime t1 = ros::WallTime::now();
  BoxPointType bbox;
  bbox.vertex_max[0] = 1e3;
  bbox.vertex_max[1] = 1e3;
  bbox.vertex_max[2] = 1e3;
  bbox.vertex_min[0] = -1e3;
  bbox.vertex_min[1] = -1e3;
  bbox.vertex_min[2] = -1e3;
  PointVector points;
  kdtree_ptr_->Box_Search(bbox, points);
  pcl::PointCloud<pcl::PointXYZ> pcd;
  pcd.points.assign(points.begin(), points.end());
  pcd.height = 1;
  pcd.width = pcd.points.size();
  if (pcd.size() > 0) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pcd, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time::now();
    response.pcd_in_bbox = msg;

    auto p3d = octree_ptr_->keyToCoord(drivable_disjoint_root_);
    geometry_msgs::Point root;
    root.x = p3d.x();
    root.y = p3d.y();
    root.z = p3d.z();
    response.root_point = root;
    int num =
        drivable_disjoint_set_->getUnionedVoxelSize(drivable_disjoint_root_);
    printf(
        "<queryAllDrivablePCDServerCallback>: get %d / %d points, timecost "
        "%.3f s\n",
        static_cast<int>(pcd.size()), num, (ros::WallTime::now() - t1).toSec());
    return true;
  }
  return false;
}

// unused. for local planning to generate costmap
bool TerrainEvaluation::queryTerrainAttributeServerCallback(
    query_terrain_attribute::Request &request,  // NOLINT
    query_terrain_attribute::Response &response) {
  std::lock_guard<std::mutex> lock_octree(
      world_representation_->globalOctomapMutex());
  ros::WallTime t1 = ros::WallTime::now();
  response.traversibility_array.data.clear();
  for (auto pc : request.point_array.points) {
    auto key = octree_ptr_->coordToKey(pc.x, pc.y, pc.z);
    auto node = octree_ptr_->search(key);
    if (!node || !octree_ptr_->isNodeOccupied(node)) {
      response.traversibility_array.data.push_back(-1);
      continue;
    }
    if (node->getCollision()) {
      response.traversibility_array.data.push_back(1 * 100);
      continue;
    }

    int traversibility = std::min(node->getTraversability() * 100, 127.0f);
    response.traversibility_array.data.push_back(traversibility);
  }

  ros::WallDuration d = ros::WallTime::now() - t1;
  printf("<queryTerrainAttributeServerCallback>: timecost: %.3fs\n", d.toSec());
  response.status = true;
  return true;
}

// for global planning to query the nearest point to the goal.
bool TerrainEvaluation::queryDrivablePointServerCallback(
    query_nearest_drivable_point::Request &request,  // NOLINT
    query_nearest_drivable_point::Response &response) {
  std::lock_guard<std::mutex> lock(kdtree_mutex_);
  if (!kdtree_initialized_ || !drivable_disjoint_root_valid_) {
    ROS_ERROR(
        "<TerrainEvaluation::queryDrivablePointServerCallback>: ikd-tree "
        "or disjoint set NOT initialized!");
    return false;
  }

  auto p = request.query_point;
  auto key = octree_ptr_->coordToKey(octomap::point3d(p.x, p.y, p.z));
  // 先判断是否在disjoint set中，标记response.success
  response.success = true;
  if (!checkDrivable(key)) {
    response.success = false;
  }

  // 在disjoint set中，则直接返回coord，并且success为true
  if (response.success == true) {
    auto p3d = octree_ptr_->keyToCoord(key);
    geometry_msgs::Point pr;
    pr.x = p3d.x();
    pr.y = p3d.y();
    pr.z = p3d.z();
    response.result_point = pr;
    printf(
        "<TerrainEvaluation::queryDrivablePointServerCallback>: disjoint_set"
        " query [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f].\n",
        p.x, p.y, p.z, pr.x, pr.y, pr.z);
    return true;
  }

  // 不在disjoint set中，success为false，查询kdtree
  mapping_module::PointVector Nearest_Points;
  std::vector<float> Point_Distance;
  kdtree_ptr_->Nearest_Search(PointType(p.x, p.y, p.z), 1, Nearest_Points,
                              Point_Distance);
  if (Nearest_Points.size() <= 0) {
    ROS_ERROR(
        "<TerrainEvaluation::queryDrivablePointServerCallback>: ikd-tree "
        "get no result for [%.3f, %.3f, %.3f].",
        p.x, p.y, p.z);
    return false;
  }

  geometry_msgs::Point pr;
  pr.x = Nearest_Points[0].x;
  pr.y = Nearest_Points[0].y;
  pr.z = Nearest_Points[0].z;
  response.result_point = pr;
  printf(
      "<TerrainEvaluation::queryDrivablePointServerCallback>: ikd-tree "
      "query [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f].\n",
      p.x, p.y, p.z, pr.x, pr.y, pr.z);
  return true;
}

void TerrainEvaluation::getPointsInKDtree(geometry_msgs::Point p,
                                          PointVector &points,
                                          float bbox_height, float bbox_size) {
  BoxPointType bbox;
  bbox.vertex_max[0] = p.x + bbox_size / 2;
  bbox.vertex_max[1] = p.y + bbox_size / 2;
  bbox.vertex_max[2] = p.z + bbox_height / 2;
  bbox.vertex_min[0] = p.x - bbox_size / 2;
  bbox.vertex_min[1] = p.y - bbox_size / 2;
  bbox.vertex_min[2] = p.z - bbox_height / 2;
  kdtree_ptr_->Box_Search(bbox, points);
}

bool TerrainEvaluation::generateCostmap(geometry_msgs::Point p) {
  // 传入中心位置、包围框大小
  float bbox_size = costmap_size_;      // 4;
  float bbox_height = costmap_height_;  // 3.8;

  // std::lock_guard<std::mutex> lock(kdtree_mutex_);
  if (!kdtree_initialized_ || !drivable_disjoint_root_valid_) {
    ROS_ERROR(
        "<TerrainEvaluation::generateCostmap>: ikd-tree "
        "or disjoint set NOT initialized!");
    return false;
  }
  mapping_module::PointVector Nearest_Points;
  getPointsInKDtree(p, Nearest_Points, bbox_height, bbox_size);
  if (Nearest_Points.size() <= 0) {
    ROS_ERROR(
        "<TerrainEvaluation::generateCostmap>: ikd-tree "
        "get no result for [%.3f, %.3f, %.3f].",
        p.x, p.y, p.z);
    return false;
  }
  // std::cout << "<TerrainEvaluation::generateCostmap>: get "
  //           << Nearest_Points.size() << " points in kd-tree" << std::endl;

  // 构建costmap地图
  float res = octree_ptr_->getResolution();
  int width = bbox_size / res;
  int height = bbox_size / res;

  // costmap_.info.origin.orientation = q;  // 忽略q
  costmap_.info.origin.position = p;  // 相当于tf
  costmap_.info.origin.position.x -= bbox_size / 2;
  costmap_.info.origin.position.y -= bbox_size / 2;
  costmap_.info.width = width;
  costmap_.info.height = height;
  costmap_.info.map_load_time = ros::Time::now();
  costmap_.info.resolution = res;
  costmap_.data.clear();
  costmap_.data.resize(height * width, 101);  // map: green
  costmap_.header.stamp = ros::Time::now();
  costmap_.header.frame_id = map_frame_;
  elevation_map_ = costmap_;
  elevation_map_.data.resize(height * width, -101);  // costmap: red
  traversibility_map_ = elevation_map_;

  // 2D去重，保留最高点
  octomap::KeySet keyset;
  int offset_x = width / 2;
  int offset_y = height / 2;
  std::unordered_set<int> idx_set;
  for (auto pt : Nearest_Points) {
    auto node = octree_ptr_->search(octree_ptr_->coordToKey(pt.x, pt.y, pt.z));
    if (!node) continue;
    // 去中心后的index，再加上偏移，避免为负
    int xi = (pt.x - p.x) / res + offset_x;
    int yi = (pt.y - p.y) / res + offset_y;
    int zi = (pt.z - p.z) / res;

    if (xi >= width || xi < 0) continue;
    if (yi >= height || yi < 0) continue;
    if (zi >= 100 || zi <= -100) continue;

    int idx = yi * width + xi;  // idx = y * width + x
    if (idx_set.find(idx) != idx_set.end()) {
      // 已经更新过，而且高度比原来低，跳过
      if (zi < elevation_map_.data[idx]) continue;
    } else {
      idx_set.insert(idx);
    }

    costmap_.data[idx] = 0;         // map: 0是可通行, white
    elevation_map_.data[idx] = zi;  // costmap: 0 trasparent, >0 blue, <0 yellow

    float traversibility = node->getTraversability();
    traversibility_map_.data[idx] = std::min(
        std::max(traversibility * 100, (float)0.0), (float)100.0);  // NOLINT
  }

  costmap_publisher_.publish(costmap_);
  elevation_map_publisher_.publish(elevation_map_);
  traversibility_map_publisher_.publish(traversibility_map_);
  return true;
}

}  // namespace mapping_module
