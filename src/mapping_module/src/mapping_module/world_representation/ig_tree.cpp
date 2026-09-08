/*
 * Created on Thu Jan 21 2021
 *
 * Copyright (c) 2021 HITSZ-NRSL
 * Copyright (c) 2021 ETH Zurich
 *
 * Original Author: Stefan Isler, islerstefan@bluewin.ch
 * (ETH Zurich / Robotics and Perception Group, University of Zurich,
 * Switzerland)
 *
 * Rewritter: Ming Cao, Yuxiang Li, Haoyao Chen
 */

#include "mapping_module/world_representation/ig_tree.h"

#include <cmath>
#include <random>

#include "mapping_module/utils/octomap_utils.h"

namespace octomap {

double IgTreeNode::getMeanChildLogOdds() const {
  double mean = 0;
  char c = 0;
  if (children != NULL) {
    for (unsigned int i = 0; i < 8; i++) {
      IgTreeNode *child = static_cast<IgTreeNode *>(children[i]);
      if (child != NULL) {
        mean += child->getOccupancy();
        c++;
      }
    }
  }
  if (c) mean /= static_cast<double>(c);

  return log(mean / (1 - mean));
}

float IgTreeNode::getMaxChildLogOdds() const {
  float max = -std::numeric_limits<float>::max();
  if (children != NULL) {
    for (unsigned int i = 0; i < 8; i++) {
      IgTreeNode *child = static_cast<IgTreeNode *>(children[i]);
      if (child != NULL) {
        float l = child->getLogOdds();
        if (l > max) max = l;
      }
    }
  }
  return max;
}

void IgTreeNode::addValue(const float &logOdds) { value += logOdds; }

std::istream &IgTreeNode::readData(std::istream &s) {
  s.read(reinterpret_cast<char *>(&this->value), sizeof(value));  // occupancy
  s.read(reinterpret_cast<char *>(&this->color),
         sizeof(ColorOcTreeNode::Color));
  s.read(reinterpret_cast<char *>(&has_no_measurement_),
         sizeof(has_no_measurement_));
  s.read(reinterpret_cast<char *>(&is_frontier_), sizeof(is_frontier_));
  s.read(reinterpret_cast<char *>(&mu_), sizeof(mu_));
  s.read(reinterpret_cast<char *>(&pt_cnt_), sizeof(pt_cnt_));

  s.read(reinterpret_cast<char *>(&tsdf_distance_), sizeof(tsdf_distance_));
  s.read(reinterpret_cast<char *>(&tsdf_weight_), sizeof(tsdf_weight_));
  s.read(reinterpret_cast<char *>(&roughness_), sizeof(roughness_));
  s.read(reinterpret_cast<char *>(&slope_), sizeof(slope_));
  s.read(reinterpret_cast<char *>(&sparsity_), sizeof(sparsity_));
  s.read(reinterpret_cast<char *>(&height_diff_), sizeof(height_diff_));
  s.read(reinterpret_cast<char *>(&traversability_), sizeof(traversability_));
  s.read(reinterpret_cast<char *>(&collision_), sizeof(collision_));
  s.read(reinterpret_cast<char *>(&fuse_eigen_info_), sizeof(fuse_eigen_info_));
  return s;
}

std::ostream &IgTreeNode::writeData(std::ostream &s) const {
  s.write(reinterpret_cast<const char *>(&this->value), sizeof(value));
  s.write(reinterpret_cast<const char *>(&this->color),
          sizeof(ColorOcTreeNode::Color));
  s.write(reinterpret_cast<const char *>(&has_no_measurement_),
          sizeof(has_no_measurement_));
  s.write(reinterpret_cast<const char *>(&is_frontier_), sizeof(is_frontier_));
  s.write(reinterpret_cast<const char *>(&mu_), sizeof(mu_));
  s.write(reinterpret_cast<const char *>(&pt_cnt_), sizeof(pt_cnt_));

  s.write(reinterpret_cast<const char *>(&tsdf_distance_),
          sizeof(tsdf_distance_));
  s.write(reinterpret_cast<const char *>(&tsdf_weight_), sizeof(tsdf_weight_));
  s.write(reinterpret_cast<const char *>(&roughness_), sizeof(roughness_));
  s.write(reinterpret_cast<const char *>(&slope_), sizeof(slope_));
  s.write(reinterpret_cast<const char *>(&sparsity_), sizeof(sparsity_));
  s.write(reinterpret_cast<const char *>(&height_diff_), sizeof(height_diff_));
  s.write(reinterpret_cast<const char *>(&traversability_),
          sizeof(traversability_));
  s.write(reinterpret_cast<const char *>(&collision_), sizeof(collision_));
  s.write(reinterpret_cast<const char *>(&fuse_eigen_info_),
          sizeof(fuse_eigen_info_));
  return s;
}

void IgTreeNode::insertPoint(const octomap::point3d &pt) {
  if (pt_cnt_ > 30) return;
  Eigen::Vector3f diff_pt_eigen(pt.x(), pt.y(), pt.z());
  float factor = 1 / static_cast<float>(pt_cnt_ + 1);
  sigma_ = pt_cnt_ * factor * sigma_ + pt_cnt_ * factor * factor *
                                           (diff_pt_eigen - mu_) *
                                           (diff_pt_eigen - mu_).transpose();
  mu_ = mu_ + factor * (diff_pt_eigen - mu_);
  ++pt_cnt_;
}

void IgTreeNode::deletePoint(const octomap::point3d &pt) {
  Eigen::Vector3f pt_eigen = octomap::octomapPointToEigenVector<float>(pt);
  float factor = 1 / static_cast<float>(pt_cnt_ - 1);
  mu_ = mu_ + factor * (mu_ - pt_eigen);
  sigma_ = pt_cnt_ * factor * sigma_ - pt_cnt_ * factor * factor *
                                           (pt_eigen - mu_) *
                                           (pt_eigen - mu_).transpose();
  --pt_cnt_;
}

/*************************** IgTree *****************************/

IgTree::Config::Config()
    : resolution_m(0.1),
      occupancy_threshold(0.5),
      hit_probability(0.9),
      miss_probability(0.2),
      clamping_threshold_min(0.12),
      clamping_threshold_max(0.97) {}

IgTree::IgTree(double resolution_m)
    : ::octomap::OccupancyOcTreeBase<IgTreeNode>(resolution_m) {
  config_.resolution_m = resolution_m;
  // ::octomap::AbstractOcTree::registerTreeType(this);
  igTreeMemberInit.ensureLinking();
  updateOctreeConfig();
}

IgTree::IgTree(Config config)
    : ::octomap::OccupancyOcTreeBase<IgTreeNode>(config.resolution_m),
      config_(config) {
  igTreeMemberInit.ensureLinking();
  updateOctreeConfig();
}

void IgTree::allocIgTreeNodeChildren(octomap::IgTreeNode *node) {
  node->children = new AbstractOcTreeNode *[8];
  for (unsigned int i = 0; i < 8; i++) {
    node->children[i] = NULL;
  }
}

octomap::IgTreeNode *IgTree::createIgTreeNodeChild(octomap::IgTreeNode *node,
                                                   unsigned int childIdx) {
  assert(childIdx < 8);
  if (node->children == NULL) {
    allocIgTreeNodeChildren(node);
  }
  assert(node->children[childIdx] == NULL);
  octomap::IgTreeNode *newNode = new octomap::IgTreeNode();
  node->children[childIdx] = static_cast<AbstractOcTreeNode *>(newNode);

  tree_size++;
  size_changed = true;

  return newNode;
}

void IgTree::copyNodeRecurs(std::shared_ptr<octomap::IgTree> tree,
                            octomap::IgTreeNode *src_node,
                            octomap::IgTreeNode *copy_node) {
  // 获取this中该节点的children
  for (unsigned int i = 0; i < 8; i++) {
    if (this->nodeChildExists(src_node, i)) {
      octomap::IgTreeNode *srcChildNode = this->getNodeChild(src_node, i);
      octomap::IgTreeNode *copyChildNode =
          tree->createNodeChild(copy_node, i);  // createNodeChild
      copyChildNode->copyData(*srcChildNode);
      this->copyNodeRecurs(tree, srcChildNode, copyChildNode);
    }
  }
}
// deleteIgTreeNodeRecurs
std::shared_ptr<octomap::IgTree> IgTree::deepClone() {
  if (!this->getRoot() || !this->getRoot()->hasChildren()) return nullptr;
  std::shared_ptr<octomap::IgTree> tree;
  tree.reset(new octomap::IgTree(this->config()));
  tree->tree_size = 0;

  tree->root = new octomap::IgTreeNode();
  tree->root->copyData(*this->root);
  this->copyNodeRecurs(tree, this->root, tree->root);

  tree->tree_size = calcNumNodes();
  return tree;
}

void IgTree::updateOctreeConfig() {
  setOccupancyThres(config_.occupancy_threshold);
  setProbHit(config_.hit_probability);
  setProbMiss(config_.miss_probability);
  setClampingThresMin(config_.clamping_threshold_min);
  setClampingThresMax(config_.clamping_threshold_max);
}

bool IgTree::isNodeUnknown(octomap::IgTreeNode *node_ptr) {
  if (node_ptr == NULL || node_ptr->hasMeasurement() == false)
    return true;
  else
    return false;
}

void IgTree::expandNode(IgTreeNode *node) {
  assert(!nodeHasChildren(node));

  for (unsigned int k = 0; k < 8; k++) {
    IgTreeNode *child = createNodeChild(node, k);
    child->copyData(*node);
  }
}

bool IgTree::pruneNode(IgTreeNode *node) {
  // if (!isNodeCollapsible(node) || this->isNodeOccupied(node)
  //   || node->isFrontier())
  //   return false;

  // // set value to children's values (all assumed equal)
  // node->copyData(*(getNodeChild(node, 0)));

  // for (unsigned int i = 0; i < 8; i++) {
  //   if (getNodeChild(node, i)->hasMeasurement()) {
  //     node->updateHasMeasurement(true);
  //     break;
  //   }
  // }

  // node->updateHasMeasurement(true);

  // if (node->hasMeasurement()) {
  //   node->updateOccDist(node->getMinChildOccDist());
  //   node->setMaxDist(node->getMaxChildDist());

  //   if (node->isColorSet()) {
  //     node->setColor(node->getAverageChildColor());
  //   }
  // }

  // // delete children
  // for (unsigned int i = 0; i < 8; i++) {
  //   deleteNodeChild(node, i);
  // }
  // delete[] node->children;
  // node->children = NULL;

  return false;
}

IgTreeNode *IgTree::setNodeColor(const ::octomap::OcTreeKey &key, uint8_t r,
                                 uint8_t g, uint8_t b) {
  IgTreeNode *n = search(key);
  if (n != 0) {
    n->setColor(r, g, b);
  }
  return n;
}

void IgTree::rayTrace(const octomap::point3d &sensor_origin,
                      float sensorMaxRange, const octomap::point3d &point,
                      octomap::KeySet *free_cells,
                      octomap::KeySet *occupied_cells, int update_free,
                      octomap::Boundingbox &local_bbox) {
  point3d dir = point - sensor_origin;
  float dis = dir.norm();
  dir.normalize();
  point3d truncted_origin;

  if (sensorMaxRange <= 0.0 || dis <= sensorMaxRange) {
    if (truncted_length_ <= 0 || dis < truncted_length_) {
      truncted_origin = sensor_origin;
    } else {
      switch (update_free) {
        case 1: {  // 1: all free voxel
          truncted_origin = sensor_origin + dir * truncted_length_;
        } break;
        case 2: {  //  2: update free voxels only 1m near occ voxels
          truncted_origin = point - dir * truncted_length_;
          break;
        }
      }
    }
    // Cast a ray to compute the free cells.
    key_ray_.reset();
    if (update_free != 0 &&  // 0: no free
        this->computeRayKeys(truncted_origin, point, key_ray_)) {
      /*   do not update nodes which are occupied*/
      for (const auto &key : key_ray_) {
        octomap::IgTreeNode *node = this->search(key);
        if (!local_bbox.isReset() &&  // Ignore the points outside
            !local_bbox.ifContain(this->keyToCoord(key)))
          continue;
        if (node == NULL) {
          free_cells->insert(key);
        } else if (!this->isNodeOccupied(node)) {
          free_cells->insert(key);
        }
      }
      /******  update all nodes to free ****/
      // free_cells->insert(key_ray_.begin(), key_ray_.end());
    }
    // Mark endpoing as occupied.
    octomap::OcTreeKey key;
    if (this->coordToKeyChecked(point, key)) {
      if (local_bbox.isReset()  // Ignore the points outside
          || local_bbox.ifContain(this->keyToCoord(key)))
        occupied_cells->insert(key);
    }
  } else if (update_free == 1) {  // 1: all free
    // If the ray is longer than the max range, just update free space.
    octomap::point3d new_end = sensor_origin + dir * sensorMaxRange;

    key_ray_.reset();
    if (this->computeRayKeys(sensor_origin, new_end, key_ray_)) {
      /*   do not update nodes which are occupied*/
      for (const auto &key : key_ray_) {
        octomap::IgTreeNode *node = this->search(key);
        if (!local_bbox.isReset() &&  // Ignore the points outside
            !local_bbox.ifContain(this->keyToCoord(key)))
          continue;
        if (node == NULL) {
          free_cells->insert(key);
        } else if (!this->isNodeOccupied(node)) {
          free_cells->insert(key);
        }
      }
      /******  update all nodes to free ****/
      // free_cells->insert(key_ray_.begin(), key_ray_.end());
    }
  }
}

void IgTree::updateOccupancy(octomap::KeySet *free_cells,
                             octomap::KeySet *occupied_cells) {
  // Mark occupied cells.
  for (octomap::KeySet::iterator it = occupied_cells->begin(),
                                 end = occupied_cells->end();
       it != end; it++) {
    this->updateNode(*it, true);

    // Remove any occupied cells from free cells - assume there are far fewer
    // occupied cells than free cells, so this is much faster than checking on
    // every free cell.
    if (free_cells->find(*it) != free_cells->end()) {
      free_cells->erase(*it);
    }
  }

  // Mark free cells.
  for (octomap::KeySet::iterator it = free_cells->begin(),
                                 end = free_cells->end();
       it != end; ++it) {
    this->updateNode(*it, false);
  }
}

std::pair<KeySet, std::vector<IgTreeNode *>> IgTree::insertPointCloud(
    const octomap::Pointcloud &valid_pc, const octomap::point3d &origin,
    octomap::Boundingbox &local_bbox, int update_free, float max_range) {
  KeySet free_cells, occupied_cells;
  std::vector<IgTreeNode *> occupied_nodes;
  octomap::OcTreeKey key;
  if (!this->coordToKeyChecked(origin, key))
    return std::make_pair(occupied_cells, occupied_nodes);

  std::unordered_map<octomap::OcTreeKey, std::vector<point3d>,
                     octomap::OcTreeKey::KeyHash>
      key_point_set;
  octomap::Boundingbox update_bbox;
  for (auto it : valid_pc) {
    // First, check if we've already checked this.
    if (!this->coordToKeyChecked(it, key)) continue;
    key_point_set[key].push_back(it);  // 保存同一体素中的点
    // if key is in the set, then the result is 0
    if (occupied_cells.find(key) == occupied_cells.end()) {  // lazy update
      // Check if this is within the allowed sensor range.
      rayTrace(origin, max_range, it, &free_cells, &occupied_cells, update_free,
               local_bbox);
      update_bbox.insertPoint(it);
    }
  }
  // Apply the new free cells and occupied cells from
  updateOccupancy(&free_cells, &occupied_cells);

  // Update voxel info
  for (KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end();
       it != end; ++it) {
    auto voxel_key = *it;
    IgTreeNode *voxel = search(voxel_key);
    occupied_nodes.push_back(voxel);
    for (auto &pt : key_point_set[voxel_key]) {
      voxel->insertPoint(pt);
      if (enable_tsdf_) {
        voxel->updateTsdf(origin, pt, keyToCoord(voxel_key),
                          this->config().resolution_m);
      }
    }
  }
  local_bbox = update_bbox;
  return std::make_pair(occupied_cells, occupied_nodes);
}

// old. unused.
std::pair<KeySet, std::vector<IgTreeNode *>> IgTree::insertPointCloud(
    const octomap::Pointcloud &valid_pc, const octomap::point3d &origin,
    octomap::Boundingbox &local_bbox, int update_free) {
  KeySet free_cells, occupied_cells;
  std::vector<IgTreeNode *> occupied_nodes;
  OcTreeKey origin_key;
  if (!this->coordToKeyChecked(origin, origin_key))
    return std::make_pair(occupied_cells, occupied_nodes);
  std::vector<std::pair<octomap::point3d, octomap::point3d>> bbox_surfaces;
  if (!local_bbox.isReset()) {  // valid bbox
    // std::cout << "<IgTree::insertPointCloud>: " << local_bbox << std::endl;
    octomap::point3d c = local_bbox.center();
    octomap::point3d s = local_bbox.size();

    bbox_surfaces.push_back(
        std::make_pair(octomap::point3d(0, 0, 1),
                       octomap::point3d(c.x(), c.y(), c.z() + s.z() * 0.5)));
    bbox_surfaces.push_back(
        std::make_pair(octomap::point3d(0, 0, 1),
                       octomap::point3d(c.x(), c.y(), c.z() - s.z() * 0.5)));
    bbox_surfaces.push_back(
        std::make_pair(octomap::point3d(0, 1, 0),
                       octomap::point3d(c.x(), c.y() + s.y() * 0.5, c.z())));
    bbox_surfaces.push_back(
        std::make_pair(octomap::point3d(0, 1, 0),
                       octomap::point3d(c.x(), c.y() - s.y() * 0.5, c.z())));
    bbox_surfaces.push_back(
        std::make_pair(octomap::point3d(1, 0, 0),
                       octomap::point3d(c.x() + s.x() * 0.5, c.y(), c.z())));
    bbox_surfaces.push_back(
        std::make_pair(octomap::point3d(1, 0, 0),
                       octomap::point3d(c.x() - s.x() * 0.5, c.y(), c.z())));
  }

  /* During inserting point cloud, the voxels can't be queried.*/
  std::unordered_map<octomap::OcTreeKey, std::vector<point3d>,
                     octomap::OcTreeKey::KeyHash>
      key_point_set;
  KeySet valid_keys;
  KeyRay key_ray_temp;
  octomap::Boundingbox update_bbox;  // 用于记录当前帧的包围框
  //   std::cout << "<IgTree::insertPointCloud>: valid_pc.size = "
  //   << valid_pc.size() << std::endl;

  for (size_t i = 0; i < valid_pc.size(); ++i) {
    point3d point = valid_pc[i];
    OcTreeKey key;
    if (!this->coordToKeyChecked(point, key)) continue;  // 截断前为key

    point3d dir = point - origin;
    float dis = dir.norm();
    dir /= dir.norm();

    bool is_inbbox = false;
    if (!local_bbox.isReset()) {  // valid bbox
      is_inbbox = local_bbox.ifContainApprox(point);
      if (!is_inbbox &&  // 在局部地图外面，而且是更新all free
          update_free == 1) {  // 则截断到框上
        octomap::point3d cross_point;
        if (octomap::rayPlaneIntersection(
                dir, origin, bbox_surfaces, local_bbox,
                cross_point))  // get the cross point from the ray to bbox
          point = cross_point;
      }
    }

    OcTreeKey point_key;
    if (!this->coordToKeyChecked(point, point_key))
      continue;  // 截断后为point_key
    if (valid_keys.find(point_key) != valid_keys.end()) {  // 在valid_keys里面
      // lazy update: 同一体素内的点只更新一次
      key_point_set[point_key].push_back(point);  // 保存同一体素中的点
      continue;
    }
    valid_keys.insert(point_key);
    update_bbox.insertPoint(point);

    point3d truncted_origin;
    if (truncted_length_ <= 0 || dis < truncted_length_) {
      truncted_origin = origin;
    } else {
      switch (update_free) {
        case 1: {  // 1: all free voxel
          truncted_origin = origin + dir * truncted_length_;
        } break;
        case 2: {  //  2: update free voxels only 1m near occ voxels
          truncted_origin = point - dir * truncted_length_;
          break;
        }
      }
      // if (!this->coordToKeyChecked(truncted_origin, key)) continue;
    }

    if (update_free != 0) {  //       // # 0: no free voxel,
      if (this->computeRayKeys(truncted_origin, point, key_ray_temp)) {
        free_cells.insert(key_ray_temp.begin(),
                          key_ray_temp.end());  // 其中可能有occ、unknown的
      }
    }

    if (local_bbox.isReset() || is_inbbox) {
      occupied_cells.insert(this->coordToKey(point));
    }
  }
  if (local_bbox.isReset()) {
    local_bbox = update_bbox;
  }

  for (KeySet::iterator it = free_cells.begin(), end = free_cells.end();
       it != end; ++it) {
    OcTreeKey voxel_key = *it;
    if (occupied_cells.find(voxel_key) ==
        occupied_cells.end()) {  // 不在occupied_cells中
      IgTreeNode *voxel = search(*it);
      if (voxel == NULL) {
        voxel = this->updateNode(voxel_key, false);
        voxel->updateHasMeasurement(true);
      } else {
        if (isNodeOccupied(voxel))
          continue;  // 去掉之后，地面会更新为free。加上之后跳过占据格子，free射线会被截断???
        if (!voxel->hasMeasurement()) {
          float logodds_first_miss =
              octomap::logodds(this->config().miss_probability);
          voxel->setLogOdds(logodds_first_miss);
          voxel->updateHasMeasurement(true);
        } else {
          this->updateNode(voxel_key, false);
          voxel->updateHasMeasurement(true);
        }
      }
    }
  }

  // std::cout << "<IgTree::insertPointCloud>: free_cells.size = "
  //           << free_cells.size()
  //           << ", occupied_cells.size = "
  //           << occupied_cells.size() << std::endl;

  for (KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end();
       it != end; ++it) {
    OcTreeKey voxel_key = *it;
    IgTreeNode *voxel = search(voxel_key);
    occupied_nodes.push_back(voxel);

    if (voxel == NULL) {
      voxel = this->updateNode(voxel_key, true);
      voxel->updateHasMeasurement(true);
    } else {
      if (!voxel->hasMeasurement()) {
        float logodds_first_hit =
            octomap::logodds(this->config().hit_probability);
        voxel->setLogOdds(logodds_first_hit);
        voxel->updateHasMeasurement(true);
      } else {
        this->updateNode(voxel_key, true);
      }
    }
    for (auto &pt : key_point_set[voxel_key]) {
      voxel->insertPoint(pt);
      voxel->updateTsdf(origin, pt, keyToCoord(voxel_key),
                        this->config().resolution_m);
    }
  }
  return std::make_pair(occupied_cells, occupied_nodes);
}

void IgTree::pullOutPointCloud(const octomap::Pointcloud &pc,
                               const octomap::point3d &origin) {
  for (int i = 0; i < pc.size(); ++i) {
    IgTreeNode *tree_node = this->search(pc[i]);
    if (tree_node) {
      if (tree_node->pointCount() == 1) {
        this->deleteNode(pc[i]);
      } else {
        tree_node->deletePoint(pc[i]);
      }
    }
  }
}

void IgTree::deleteNodesOutsideBbox(const octomap::Boundingbox &bbox,
                                    float delta) {
  std::vector<octomap::Boundingbox> outer_bboxes;
  octomap::Boundingbox tmp_bbox;
  tmp_bbox.reset();  // upper_bbox
  tmp_bbox.insertPoint(octomap::point3d(
      bbox.maxX() + delta, bbox.maxY() + delta, bbox.maxZ() + delta));
  tmp_bbox.insertPoint(
      octomap::point3d(bbox.minX() - delta, bbox.minY() - delta, bbox.maxZ()));
  outer_bboxes.push_back(tmp_bbox);

  tmp_bbox.reset();  // lower_bbox
  tmp_bbox.insertPoint(
      octomap::point3d(bbox.maxX() + delta, bbox.maxY() + delta, bbox.minZ()));
  tmp_bbox.insertPoint(octomap::point3d(
      bbox.minX() - delta, bbox.minY() - delta, bbox.minZ() - delta));
  outer_bboxes.push_back(tmp_bbox);

  tmp_bbox.reset();  // rear_bbox
  tmp_bbox.insertPoint(
      octomap::point3d(bbox.minX(), bbox.maxY() + delta, bbox.maxZ()));
  tmp_bbox.insertPoint(
      octomap::point3d(bbox.minX() - delta, bbox.minY() - delta, bbox.minZ()));
  outer_bboxes.push_back(tmp_bbox);

  tmp_bbox.reset();  // front_bbox
  tmp_bbox.insertPoint(
      octomap::point3d(bbox.maxX() + delta, bbox.maxY() + delta, bbox.maxZ()));
  tmp_bbox.insertPoint(
      octomap::point3d(bbox.maxX(), bbox.minY() - delta, bbox.minZ()));
  outer_bboxes.push_back(tmp_bbox);

  tmp_bbox.reset();  // left_bbox
  tmp_bbox.insertPoint(octomap::point3d(bbox.maxX(), bbox.minY(), bbox.maxZ()));
  tmp_bbox.insertPoint(
      octomap::point3d(bbox.minX(), bbox.minY() - delta, bbox.minZ()));
  outer_bboxes.push_back(tmp_bbox);

  tmp_bbox.reset();  // right_bbox
  tmp_bbox.insertPoint(
      octomap::point3d(bbox.maxX(), bbox.maxY() + delta, bbox.maxZ()));
  tmp_bbox.insertPoint(octomap::point3d(bbox.minX(), bbox.maxY(), bbox.minZ()));
  outer_bboxes.push_back(tmp_bbox);

  std::vector<std::pair<OcTreeKey, unsigned int>> keys_to_delete;
  for (int i = 0; i < outer_bboxes.size(); i++) {
    octomap::Boundingbox bbox_tmp = outer_bboxes[i];
    bbox_tmp.enlarge(0.3);
    leaf_bbx_iterator iter_begin =
        begin_leafs_bbx(bbox_tmp.minPoint(), bbox_tmp.maxPoint());
    leaf_bbx_iterator iter_end = end_leafs_bbx();
    for (auto it = iter_begin; it != iter_end; ++it) {
      keys_to_delete.push_back(std::make_pair(it.getKey(), it.getDepth()));
    }
  }
  // int before_num = this->getNumLeafNodes();
  // std::vector<std::pair<OcTreeKey, unsigned int>> keys_to_delete;
  // // Inner nodes can be deleted with tree iterator, not leaf iterator!!!
  // // auto start = this->begin_tree(this->getTreeDepth());
  // // auto end = this->end_tree();
  // auto start = this->begin_leafs(this->getTreeDepth());
  // auto end = this->end_leafs();
  // for (auto it = start; it != end; ++it) {
  //   point3d leaf_pt = this->keyToCoord(it.getKey());
  //   // if (!bbox.ifContain(leaf_pt)) {
  //     // if (upper_bbox.ifContain(leaf_pt) || lower_bbox.ifContain(leaf_pt)
  //     ||
  //     //     rear_bbox.ifContain(leaf_pt) || front_bbox.ifContain(leaf_pt) ||
  //     //     left_bbox.ifContain(leaf_pt) || right_bbox.ifContain(leaf_pt)) {

  //     // auto node = this->search(it.getKey());
  //     // if (node && !node->hasChildren()) {
  //       keys_to_delete.push_back(std::make_pair(it.getKey(), it.getDepth()));
  //     // } else {
  //     //   keys_to_delete.push_back(
  //     //       std::make_pair(it.getKey(), this->getTreeDepth()));
  //     // }
  //   }
  // }

  for (auto &key : keys_to_delete) {
    this->deleteNode(key.first, key.second);
  }
  // int after_num = this->getNumLeafNodes();
  // printf(
  //     "<deleteNodesOutsideBbox>:  %d nodes before,  %d nodes after, delete %d
  //     " "nodes.\n", before_num, after_num, (int)keys_to_delete.size());
}

void IgTree::getNeighborhoodAtPoint27(
    const octomap::point3d &pt,
    std::vector<octomap::IgTreeNode *> *neighborhood) {
  neighborhood->clear();
  OcTreeKey pt_key = this->coordToKey(pt);
  OcTreeKey neigh_key;
  for (int i = -1; i <= 1; ++i)
    for (int j = -1; j <= 1; ++j)
      for (int k = -1; k <= 1; ++k) {
        neigh_key.k[0] = pt_key[0] + i;
        neigh_key.k[1] = pt_key[1] + j;
        neigh_key.k[2] = pt_key[2] + k;
        neighborhood->push_back(this->search(neigh_key));
      }
}

void IgTree::getNeighborhoodAtPoint7(
    const octomap::point3d &pt,
    std::vector<octomap::IgTreeNode *> *neighborhood) {
  neighborhood->clear();
  static std::vector<int> dx = {0, -1, 1, 0, 0, 0, 0};
  static std::vector<int> dy = {0, 0, 0, -1, 1, 0, 0};
  static std::vector<int> dz = {0, 0, 0, 0, 0, -1, 1};
  std::vector<octomap::OcTreeKey> neigh_keys;
  OcTreeKey key = this->coordToKey(pt);
  for (int i = 0; i < dx.size(); ++i) {
    octomap::OcTreeKey neigh_key;
    neigh_key[0] = dx[i] + key[0];
    neigh_key[1] = dy[i] + key[1];
    neigh_key[2] = dz[i] + key[2];
    neigh_keys.push_back(neigh_key);
  }
  for (int i = 0; i < neigh_keys.size(); ++i) {
    IgTreeNode *neigh_node = this->search(neigh_keys[i]);
    neighborhood->push_back(neigh_node);
  }
}

void IgTree::getNeighborhoodAtPoint1(
    const octomap::point3d &pt,
    std::vector<octomap::IgTreeNode *> *neighborhood) {
  neighborhood->clear();
  neighborhood->push_back(this->search(pt));
}

IgTree::StaticMemberInitializer IgTree::igTreeMemberInit;
};  // namespace octomap
