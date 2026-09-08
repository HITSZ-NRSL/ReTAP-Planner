/*
 * Created on Thu Jan 21 2021
 *
 * Copyright (c) 2021 HITSZ-NRSL
 * All rights reserved
 *
 * Author: Stefan Isler, islerstefan@bluewin.ch
 * (ETH Zurich / Robotics and Perception Group, University of Zurich,
 * Switzerland)
 * Modifier: EpsAvlc
 */
#pragma once

#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mapping_module/utils/bounding_box.h"
#include "mapping_module/utils/disjoint_set.h"
// #include "mapping_module/utils/octree_hash.h"

namespace octomap {

class IgTree;

class EigenInfo {
 public:
  float max_eigen_val = 0;
  float mid_eigen_val = 0;
  float min_eigen_val = 0;
  Eigen::Vector3f max_eigen_vec;
  Eigen::Vector3f mid_eigen_vec;
  Eigen::Vector3f min_eigen_vec;

  EigenInfo() {}

  explicit EigenInfo(Eigen::Matrix3f sigma) {
    Eigen::EigenSolver<Eigen::Matrix3f> es(sigma);
    auto values = es.eigenvalues().real();
    auto vectors = es.eigenvectors().real();
    int min_idx, max_idx, mid_idx;
    min_eigen_val = 1e3;
    max_eigen_val = -1e3;

    for (int i = 0; i < values.rows(); ++i) {
      if (values[i] < min_eigen_val) {
        min_idx = i;
        min_eigen_val = values[i];
      }
      if (values[i] > max_eigen_val) {
        max_idx = i;
        max_eigen_val = values[i];
      }
    }
    for (int i = 0; i < values.rows(); ++i) {
      if (i != min_idx && i != max_idx) {
        mid_idx = i;
        mid_eigen_val = values[i];
      }
    }
    min_eigen_vec = vectors.col(min_idx);
    mid_eigen_vec = vectors.col(mid_idx);
    max_eigen_vec = vectors.col(max_idx);
  }
};

class IgTreeNode : public octomap::ColorOcTreeNode {
 public:
  friend class IgTree;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IgTreeNode()
      : ColorOcTreeNode(),
        has_no_measurement_(false),
        is_frontier_(false),
        mu_(Eigen::Vector3f(0, 0, 0)),
        pt_cnt_(0),
        tsdf_distance_(0),
        tsdf_weight_(0),
        roughness_(0),
        slope_(0),
        sparsity_(1),
        height_diff_(0),
        connected_voxel_(0),
        traversability_(0),
        fuse_eigen_info_(EigenInfo()),
        collision_(0) {}
  ~IgTreeNode() {}

  bool operator==(const IgTreeNode &rhs) const {
    return rhs.value == value &&
           rhs.has_no_measurement_ == has_no_measurement_ && rhs.color == color;
  }

  void copyData(const IgTreeNode &from) {
    // ColorOcTreeNode::copyData(from);
    value = from.value;
    color = from.color;
    has_no_measurement_ = from.has_no_measurement_;
    is_frontier_ = from.is_frontier_;
    mu_ = from.mu_;
    pt_cnt_ = from.pt_cnt_;
    tsdf_distance_ = from.tsdf_distance_;
    tsdf_weight_ = from.tsdf_weight_;
    roughness_ = from.roughness_;
    slope_ = from.slope_;
    sparsity_ = from.sparsity_;
    height_diff_ = from.height_diff_;
    connected_voxel_ = from.connected_voxel_;
    traversability_ = from.traversability_;
    fuse_eigen_info_ = from.fuse_eigen_info_;
    collision_ = from.collision_;
  }
  // -- node occupancy  ----------------------------

  /// \return occupancy probability of node
  inline double getOccupancy() const { return ::octomap::probability(value); }

  /// \return log odds representation of occupancy probability of node
  inline float getLogOdds() const { return value; }
  /// sets log odds occupancy of node
  inline void setLogOdds(float l) { value = l; }

  /**
   * @return mean of all children's occupancy probabilities, in log odds
   */
  double getMeanChildLogOdds() const;

  /**
   * @return maximum of children's occupancy probabilities, in log odds
   */
  float getMaxChildLogOdds() const;

  /**
   * @brief adds p to the node's logOdds value (with no boundary / threshold
   *        checking!)
   *
   * @param p
   */
  void addValue(const float &p);

  /**
   * @brief whether this node has been measured or not. (Not used)
   *
   * @return true
   * @return false
   */
  bool hasMeasurement() { return !has_no_measurement_; }
  /**
   * @brief Not uesd
   *
   * @param hasMeasurement
   */
  void updateHasMeasurement(bool hasMeasurement) {
    has_no_measurement_ = !hasMeasurement;
  }

  /**
   * @brief This function is a serialization function. It is VITAL for octomap
   * rivz plugins' right visualization.
   *
   * @param [in] s input stream
   * @return std::istream& return the stream
   */
  std::istream &readData(std::istream &s);

  /**
   * @brief This function is a serialization function. It is VITAL for octomap
   * rivz plugins' right visualization. Function readData and writeData should
   * have the same order of reading and writing data.
   *
   * @param s
   * @return std::ostream&
   */
  std::ostream &writeData(std::ostream &s) const;

  /**
   * @brief Return if a voxel is a frontier
   *
   * @return true if this voxel is a frontier.
   * @return false if this voxel is not a frontier
   */
  bool isFrontier() { return is_frontier_; }

  /**
   * @brief Set frontier tag
   *
   * @param [in] is_frontier boolean.
   */
  void setFrontier(bool is_frontier) { is_frontier_ = is_frontier; }

  /**
   * @brief
   *
   * @param pt
   */
  void insertPoint(const octomap::point3d &pt);

  void deletePoint(const octomap::point3d &pt);

  Eigen::Vector3f mu() { return mu_; }
  void setMu(Eigen::Vector3f vec) { mu_ = vec; }

  octomap::point3d muOcto() {
    octomap::point3d pt;
    pt.x() = mu_.x();
    pt.y() = mu_.y();
    pt.z() = mu_.z();
    return pt;
  }

  Eigen::Matrix3f cov() { return sigma_; }

  int pointCount() const { return pt_cnt_; }

  void setFuseMu(Eigen::Vector3f vec) { fuse_mu_ = vec; }

  Eigen::Vector3f getFuseMu() { return fuse_mu_; }

  void setFuseSigma(Eigen::Matrix3f sigma) { fuse_sigma_ = sigma; }

  void setFuseEigen() {
    fuse_eigen_info_ = EigenInfo(fuse_sigma_);
    // 矫正法向量方向
    // point3d dir = getObservePoint() - muOcto();
    // if (fuse_eigen_info_.min_eigen_vec.dot(
    //         Eigen::Vector3f(dir.x(), dir.y(), dir.z())) < 0) {
    //   fuse_eigen_info_.min_eigen_vec *= -1;  // 方向取反
    // }
  }

  void setFuseEigen(EigenInfo ei) { fuse_eigen_info_ = ei; }

  EigenInfo getFuseEigen() { return fuse_eigen_info_; }

  void setFuseCount(int val) { fuse_pt_cnt_ = val; }

  int getFuseCount() { return fuse_pt_cnt_; }

  float getSparsity() { return sparsity_; }

  float getRoughness() { return roughness_; }

  float getSlope() { return slope_; }

  float getTraversability() { return traversability_; }

  void setSparsity(float val) { sparsity_ = val; }

  void setRoughness(float val) { roughness_ = val; }

  void setSlope(float val) { slope_ = val; }

  void setTraversability(float val) { traversability_ = val; }

  // void setObservePoint(point3d p) { observe_point_ = p; }

  // point3d getObservePoint() { return observe_point_; }

  float computeDistance(point3d origin, point3d point_G, point3d voxel_center) {
    point3d v_voxel_origin = voxel_center - origin;
    point3d v_point_origin = point_G - origin;

    float dist_G = v_point_origin.norm();
    // projection of a (v_voxel_origin) onto b (v_point_origin)
    float dist_G_V = v_voxel_origin.dot(v_point_origin) / dist_G;

    float sdf = static_cast<float>(dist_G - dist_G_V);
    return sdf;
  }

  void updateTsdf(point3d origin, point3d point_G, point3d voxel_center,
                  float voxel_size) {
    float sdf = computeDistance(origin, point_G, voxel_center);
    float updated_weight = 1;
    float max_weight = 10000;
    float default_truncation_distance = 2 * voxel_size;
    float dropoff_epsilon = voxel_size;
    if (sdf < -dropoff_epsilon) {
      updated_weight = (default_truncation_distance + sdf) /
                       (default_truncation_distance - dropoff_epsilon);
      updated_weight = std::max(updated_weight, 0.0f);
    }

    float new_weight = tsdf_weight_ + updated_weight;

    // it is possible to have weights very close to zero, due to the limited
    // precision of floating points dividing by this small value can cause nans
    if (new_weight < 1e-6) {
      return;
    }

    float new_sdf =
        (sdf * updated_weight + tsdf_distance_ * tsdf_weight_) / new_weight;

    // color blending is expensive only do it close to the surface
    // if (std::abs(sdf) < default_truncation_distance) {
    // 	tsdf_voxel->color = Color::blendTwoColors(tsdf_voxel->color,
    // tsdf_weight_, color, updated_weight);
    // }
    tsdf_distance_ = (new_sdf > 0.0)
                         ? std::min(default_truncation_distance, new_sdf)
                         : std::max(-default_truncation_distance, new_sdf);
    tsdf_weight_ = std::min(max_weight, new_weight);
    // std::cout << "<updateTsdf>: d=" << tsdf_distance_ << ", w=" <<
    // tsdf_weight_ << std::endl;
  }

  float getTsdf_Weight() { return tsdf_weight_; }
  float getTsdf_Distance() { return tsdf_distance_; }

  void setHeightDiff(float val) { height_diff_ = val; }
  float getHeightDiff() { return height_diff_; }

  void setConnected(bool val) { connected_voxel_ = val ? 0.5 : 1; }
  bool getConnected() { return static_cast<bool>(connected_voxel_ == 0.5); }

  void setCollision(bool val) { collision_ = val ? 1 : 0.5; }
  bool getCollision() { return static_cast<bool>(collision_ != 0.5); }

 protected:
  bool has_no_measurement_ = false;
  bool is_frontier_ = false;
  /** NDT parameters **/
  int pt_cnt_ = 0;
  Eigen::Vector3f mu_ = Eigen::Vector3f::Zero();
  Eigen::Matrix3f sigma_ = Eigen::Matrix3f::Zero();

  int fuse_pt_cnt_ = 0;
  Eigen::Vector3f fuse_mu_ = Eigen::Vector3f::Zero();
  Eigen::Matrix3f fuse_sigma_ = Eigen::Matrix3f::Zero();
  EigenInfo fuse_eigen_info_;

  float traversability_ = 0;
  float collision_ = 0;
  float sparsity_ = 1;
  float roughness_ = 0;
  float slope_ = 0;
  float height_diff_ = 0;
  float connected_voxel_ = 0;

  float tsdf_weight_ = 0;
  float tsdf_distance_ = 0;

  point3d observe_point_;
};

class IgTree : public ::octomap::OccupancyOcTreeBase<IgTreeNode> {
 public:
  typedef IgTreeNode NodeType;
  typedef std::shared_ptr<IgTree> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /*! Configuration for the IgTree
   */
  struct Config {
   public:
    Config();

   public:
    double resolution_m;  //! OcTree leaf node size, default: 0.1 [m].
    double
        occupancy_threshold;  //! Occupancy probability over which nodes are
                              //! considered occupied, default: 0.5 [range 0-1].
    double hit_probability;   //! Probability update value for hits, default 0.7
                              //! [range 0-1].
    double miss_probability;  //! Probability update value for misses, default
                              //! 0.4 [range 0-1].
    double clamping_threshold_min;  //! Min probability threshold over which the
                                    //! probability is clamped, default: 0.12,
                                    //! range [0-1].
    double clamping_threshold_max;  //! Max probability threshold over which the
                                    //! probability is clamped, default: 0.97,
                                    //! range [0-1].
  };

 public:
  // std::shared_ptr<IgTree> clone() const {
  //   return std::shared_ptr<IgTree>(new IgTree(*this));
  // }

  //! Default constructor, sets resolution of leafs
  explicit IgTree(double resolution_m);

  /*! Constructor with complete configuration
   */
  explicit IgTree(Config config);

  std::shared_ptr<IgTree> deepClone();
  void copyNodeRecurs(std::shared_ptr<octomap::IgTree> tree,
                      octomap::IgTreeNode *src_node,
                      octomap::IgTreeNode *copy_node);
  octomap::IgTreeNode *createIgTreeNodeChild(octomap::IgTreeNode *node,
                                             unsigned int childIdx);
  void allocIgTreeNodeChildren(octomap::IgTreeNode *node);
  /*! virtual constructor: creates a new object of same type
   * (Covariant return type requires an up-to-date compiler)
   */
  IgTree *create() const { return new IgTree(config_); }

  std::string getTreeType() const { return "IgTree"; }

  bool getTsdfEnable() { return enable_tsdf_; }
  void setTsdfEnable(bool value) { enable_tsdf_ = value; }

  /*! Returns the current configuration.
   */
  const Config &config() const { return config_; }

  unsigned int getTreeMaxVal() const { return tree_max_val; }

  /// update this node's occupancy according to its children's maximum occupancy
  inline void updateOccupancyChildren(IgTreeNode *node) {
    node->setLogOdds(node->getMaxChildLogOdds());  // conservative
  }

  void expandNode(IgTreeNode *node);
  bool pruneNode(IgTreeNode *node);

  /***************** Color Methods *********************/
  // set node color at given key or coordinate. Replaces previous color.
  IgTreeNode *setNodeColor(const ::octomap::OcTreeKey &key, uint8_t r,
                           uint8_t g, uint8_t b);

  IgTreeNode *setNodeColor(float x, float y, float z, uint8_t r, uint8_t g,
                           uint8_t b) {
    ::octomap::OcTreeKey key;
    if (!this->coordToKeyChecked(::octomap::point3d(x, y, z), key)) return NULL;
    return setNodeColor(key, r, g, b);
  }

  void rayTrace(const octomap::point3d &sensor_origin, float sensorMaxRange,
                const octomap::point3d &point, octomap::KeySet *free_cells,
                octomap::KeySet *occupied_cells, int update_free,
                octomap::Boundingbox &local_bbox);
  void updateOccupancy(octomap::KeySet *free_cells,
                       octomap::KeySet *occupied_cells);
  /**
   * @brief insert point cloud into octomap
   *
   * @param pc point cloud
   * @param origin sensor origin
   */
  std::pair<KeySet, std::vector<IgTreeNode *>> insertPointCloud(
      const octomap::Pointcloud &pc,     // NOLINT
      const octomap::point3d &origin,    // NOLINT
      octomap::Boundingbox &local_bbox,  // NOLINT
      int update_free);
  std::pair<KeySet, std::vector<IgTreeNode *>> insertPointCloud(
      const octomap::Pointcloud &pc,     // NOLINT
      const octomap::point3d &origin,    // NOLINT
      octomap::Boundingbox &local_bbox,  // NOLINT
      int update_free, float max_range);
  /**
   * @brief pull out point cloud from octomap
   *
   * @param pc pull out point cloud
   * @param origin sensor origin
   */
  void pullOutPointCloud(const octomap::Pointcloud &pc,
                         const octomap::point3d &origin);

  /**
   * @brief delete octree nodes outside a bounding box.
   *
   * @param bbox
   */
  void deleteNodesOutsideBbox(const octomap::Boundingbox &bbox, float delta);

  void getNeighborhoodAtPoint27(
      const octomap::point3d &pt,
      std::vector<octomap::IgTreeNode *> *neighborhood);

  void getNeighborhoodAtPoint7(
      const octomap::point3d &pt,
      std::vector<octomap::IgTreeNode *> *neighborhood);

  void getNeighborhoodAtPoint1(
      const octomap::point3d &pt,
      std::vector<octomap::IgTreeNode *> *neighborhood);

  void setTrunctedLength(double length) { truncted_length_ = length; }

 protected:
  /*! Sets octree options based on current configuration
   */
  void updateOctreeConfig();

 protected:
  Config config_;

  /**
   * Static member object which ensures that this OcTree's prototype
   * ends up in the classIDMapping only once. You need this as a
   * static member in any derived octree class in order to read .ot
   * files through the AbstractOcTree factory. You should also call
   * ensureLinking() once from the constructor.
   */
  class StaticMemberInitializer {
   public:
    StaticMemberInitializer() {
      // std::cout << "begin to register ig_tree" << std::endl;
      IgTree *tree = new IgTree(0.1);
      tree->clearKeyRays();
      AbstractOcTree::registerTreeType(tree);
      // std::cout << "register ig_tree" << std::endl;
    }

    /**
     * Dummy function to ensure that MSVC does not drop the
     * StaticMemberInitializer, causing this tree failing to register.
     * Needs to be called from the constructor of this octree.
     */
    void ensureLinking() {}
  };
  /// static member to ensure static initialization (only once)
  static StaticMemberInitializer igTreeMemberInit;

 private:
  bool isNodeUnknown(octomap::IgTreeNode *node_ptr);

  bool enable_tsdf_ = false;

  double truncted_length_ = 1;
  octomap::KeyRay key_ray_;
};

}  // namespace octomap
