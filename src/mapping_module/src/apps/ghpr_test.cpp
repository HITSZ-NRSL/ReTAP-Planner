/** Copyright 2023 Liyx
 * Hidden point removal test
 * Doesn't work
 * */
#include <math.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>
#include <chrono>
#include <string>
#include <vector>

#include "ros/ros.h"
#define BACKWARD_HAS_BFD 1
#include <backward_ros/backward.hpp>

float gammaa = 0.0005f;
/*
On the Visibility of Point Clouds:
https://openaccess.thecvf.com/content_iccv_2015/papers/Katz_On_the_Visibility_ICCV_2015_paper.pdf
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr surface_recon_visibility(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,  // NOLINT
    Eigen::Vector4f& viewpoint) {                // NOLINT
  auto startT = std::chrono::high_resolution_clock::now();
  // ------process start---------
  pcl::PointXYZ vp;
  vp.x = viewpoint[0];
  vp.y = viewpoint[1];
  vp.z = viewpoint[2];

  std::cout << "demeanPointCloud\n";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZ>);
  // Centralized by viewpoint
  pcl::demeanPointCloud(*cloud, viewpoint, *cloud_out);

  std::cout << "copyPointCloud\n";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud_out, *cloud_transform);
  Eigen::Vector3f points;
  float temp_norm = 0.0;

  std::cout << "gamma transform\n";
  // step 1: point cloud transform
  for (int i = 0; i < cloud_out->points.size(); ++i) {
    points[0] = cloud_out->points[i].x;
    points[1] = cloud_out->points[i].y;
    points[2] = cloud_out->points[i].z;
    temp_norm = points.norm();
    // kernel is d^gammaa
    cloud_transform->points[i].x =
        pow(temp_norm, -gammaa) * cloud_out->points[i].x / temp_norm;
    cloud_transform->points[i].y =
        pow(temp_norm, -gammaa) * cloud_out->points[i].y / temp_norm;
    cloud_transform->points[i].z =
        pow(temp_norm, -gammaa) * cloud_out->points[i].z / temp_norm;
  }

  std::cout << "ConvexHull \n";
  // step 2: create convex hull
  pcl::ConvexHull<pcl::PointXYZ> hull;
  hull.setInputCloud(cloud_transform);
  hull.setDimension(3);
  hull.setComputeAreaVolume(true);

  printf("reconstruct size=%d\n", (int)hull.getInputCloud()->size());  // NOLINT
  std::vector<pcl::Vertices> polygons;
  pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(
      new pcl::PointCloud<pcl::PointXYZ>);
  hull.reconstruct(*surface_hull);

  std::cout << "inverse transform\n";
  // step 3: inverse transform
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inverse(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*surface_hull, *cloud_inverse);
  Eigen::Vector3f points_i;
  float temp_norm_i = 0.0;
  float temp_z = 0.0;
  for (int i = 0; i < surface_hull->points.size(); ++i) {
    points_i[0] = surface_hull->points[i].x;
    points_i[1] = surface_hull->points[i].y;
    points_i[2] = surface_hull->points[i].z;
    temp_norm_i = points_i.norm();

    cloud_inverse->points[i].x = (pow(temp_norm_i, -1.0 / gammaa) *
                                  surface_hull->points[i].x / temp_norm_i) +
                                 viewpoint[0];
    cloud_inverse->points[i].y = (pow(temp_norm_i, -1.0 / gammaa) *
                                  surface_hull->points[i].y / temp_norm_i) +
                                 viewpoint[1];
    cloud_inverse->points[i].z = (pow(temp_norm_i, -1.0 / gammaa) *
                                  surface_hull->points[i].z / temp_norm_i) +
                                 viewpoint[2];
  }
  // ------process end---------
  auto endT = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> vis_ms = endT - startT;
  std::cout << "vis_time:" << vis_ms.count() << "ms" << std::endl;

  return cloud_inverse;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generate_test_pcd() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = -10; i <= 10; i++) {
    for (int j = -10; j <= 10; j++) {
      pcl::PointXYZ p;
      p.x = i * 0.2;
      p.y = j * 0.2;
      p.z = 0;
      cloud->push_back(p);
      p.z = -0.2;
      cloud->push_back(p);
    }
  }

  for (int i = -5; i <= 5; i++) {
    for (int j = -5; j <= 5; j++) {
      pcl::PointXYZ p;
      p.x = i * 0.2;
      p.y = j * 0.2;
      p.z = -0.4;
      cloud->push_back(p);
      p.z = -0.6;
      cloud->push_back(p);
    }
  }
  return cloud;
}

int main(int argc, char** argv) {
  backward::SignalHandling sh;
  ros::init(argc, argv, "ghpr_test");
  ros::NodeHandle nh;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cloud_out;
  std::cout << "generate_test_pcd\n";
  cloud = generate_test_pcd();
  Eigen::Vector4f vp(0, 0, 0, 0);
  std::cout << "write1\n";
  pcl::io::savePCDFile("/home/xxx/ghpr_test.pcd", *cloud);

  std::cout << "surface_recon_visibility\n";
  cloud_out = surface_recon_visibility(cloud, vp);
  std::cout << "write2\n";
  pcl::io::savePCDFile("/home/xxx/ghpr_out.pcd", *cloud_out);
  std::cout << "writeok\n";
}