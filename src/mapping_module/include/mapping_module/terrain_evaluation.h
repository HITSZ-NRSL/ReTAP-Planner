#include <std_msgs/Int32MultiArray.h>
#include <visualization_msgs/Marker.h>

#include "mapping_module/ikd-Tree/ikd_Tree_impl.h"
#include "mapping_module/query_all_drivable_pcd.h"
#include "mapping_module/query_nearest_drivable_point.h"
#include "mapping_module/query_terrain_attribute.h"
#include "mapping_module/utils/disjoint_set.h"
#include "mapping_module/world_representation/world_representation.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_broadcaster.h"
namespace mapping_module {
using PointType = pcl::PointXYZ;
using PointVector = KD_TREE<PointType>::PointVector;

class TerrainEvaluation {
 public:
  explicit TerrainEvaluation(std::shared_ptr<WorldRepresentation> wr);
  ~TerrainEvaluation();
  void updateMapTerrainAttribute(octomap::point3d update_position,
                                 octomap::Boundingbox update_bbox);
  void updateMapTerrainAttribute(octomap::KeySet &update_voxels,  // NOLINT
                                 octomap::point3d *update_position = nullptr);
  void updateVoxelTerrainAttribute(octomap::OcTreeKey &iter_key,  // NOLINT
                                   octomap::IgTreeNode *iter_node,
                                   octomap::KeySet *update_voxels = nullptr,
                                   octomap::point3d *update_position = nullptr,
                                   octomap::KeySet *neigh_voxels = nullptr);
  int updateVoxelTerrainCollision(octomap::OcTreeKey iter_key,
                                  octomap::IgTreeNode *iter_node);
  bool checkDrivable(octomap::OcTreeKey key);
  bool checkNodeConstraints(octomap::IgTreeNode *node, bool debug_msg = false);
  bool checkNeighborCells(octomap::OcTreeKey key, bool debug_msg = false);
  void updateDrivableDisjointSet(
      std::vector<octomap::OcTreeKey> &update_voxels,    // NOLINT
      std::vector<octomap::IgTreeNode *> &update_nodes,  // NOLINT
      std::vector<octomap::OcTreeKey> &revoke_voxels,    // NOLINT
      std::vector<octomap::IgTreeNode *> &revoke_nodes,  // NOLINT
      octomap::point3d update_position);
  bool queryDrivablePointServerCallback(
      query_nearest_drivable_point::Request &request,     // NOLINT
      query_nearest_drivable_point::Response &response);  // NOLINT
  bool queryAllDrivablePCDServerCallback(
      query_all_drivable_pcd::Request &request,     // NOLINT
      query_all_drivable_pcd::Response &response);  // NOLINT
  bool queryTerrainAttributeServerCallback(
      query_terrain_attribute::Request &request,  // NOLINT
      query_terrain_attribute::Response &response);
  bool generateCostmap(geometry_msgs::Point p);
  void getPointsInKDtree(geometry_msgs::Point p, PointVector &points,  // NOLINT
                         float bbox_height, float bbox_size);

  void resetTerrainRepresentation(std::shared_ptr<WorldRepresentation> wr);
  void terrainInit(Eigen::Isometry3d &current_pose);
  void terrainUpdate();
  std::thread terrain_thread_;
  void setCancel() {
    std::lock_guard<std::mutex> lock(cancel_mutex_);
    cancel_ = true;
  }

 private:
  float roughness_threshold_ = 1;
  float sparsity_threshold_ = 0.2;
  float slope_threshold_ = 0.785;
  float roughness_weight_ = 1.391;
  float sparsity_weight_ = 1;
  float slope_weight_ = 0.686;
  float traversability_weight_ = 2.3;
  int points_number_threshold_ = 20;
  float traversability_threshold_ = 0.7;
  float neighbor_radius_;
  std::shared_ptr<WorldRepresentation> world_representation_;
  std::shared_ptr<octomap::IgTree> octree_ptr_;
  std::string map_frame_;  // NOLINT
  float costmap_size_ = 4;
  float costmap_height_ = 3.8;
  float collision_neighbor_ = 17;
  float frame_height_ = 0.5;
  float filling_radius_ = 0.5;
  std::shared_ptr<DisjointSet<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash>>
      drivable_disjoint_set_;
  bool drivable_disjoint_root_valid_ = false;
  bool kdtree_initialized_ = false;
  bool costmap_enable_ = false;

  nav_msgs::OccupancyGrid costmap_, elevation_map_, traversibility_map_;
  octomap::OcTreeKey drivable_disjoint_root_;
  KD_TREE<PointType>::Ptr kdtree_ptr_;

  ros::Publisher local_drivable_pcd_publisher_, costmap_publisher_,
      elevation_map_publisher_, traversibility_map_publisher_;
  ros::ServiceServer query_nearest_drivable_point_server_;
  ros::ServiceServer query_all_drivable_pcd_server_,
      query_terrain_attribute_server_;

  bool cancel_ = false;
  std::mutex kdtree_mutex_, cancel_mutex_;
  bool checkCancel() {
    std::lock_guard<std::mutex> lock(cancel_mutex_);
    return cancel_;
  }

  tf::TransformBroadcaster tf_broadcaster_;
};
}  // namespace mapping_module
