#include <pcl/filters/convolution.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ros/ros.h>
void VoxelGridDownsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudout,
                         float leaf_size) {
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);  // 每个体素块的xyz值
  filter.filter(*cloudout);
  std::cout << "<VoxelGridDownsample>: cloud size: " << cloud->size()
            << ", after downsample: " << cloudout->size() << std::endl;
}

bool MedianSmooth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudout,
                  const int& size) {
  if (cloud == nullptr) return false;
  if (cloud->points.size() < 10) return false;

  pcl::MedianFilter<pcl::PointXYZ> median_filter;
  median_filter.setInputCloud(cloud);
  median_filter.setWindowSize(size);
  median_filter.filter(*cloudout);

  return true;
}

/// 使用高斯滤波进行点云平滑
bool GaussianSmooth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudout,
                    const float& threshold, const float& cigma,
                    const float& radius) {
  if (cloud == nullptr) return false;
  if (cloud->points.size() < 10) return false;

  pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(
      new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
  kernel->setSigma(cigma);  // 4
  kernel->setThresholdRelativeToSigma(4);
  kernel->setThreshold(threshold);  // 0.05
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  kdtree->setInputCloud(cloud);
  pcl::filters::Convolution3D<
      pcl::PointXYZ, pcl::PointXYZ,
      pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>>
      convolution;
  convolution.setKernel(*kernel);
  convolution.setInputCloud(cloud);
  convolution.setSearchMethod(kdtree);
  convolution.setRadiusSearch(radius);  // 0.02
  convolution.setNumberOfThreads(10);   // important! Set Thread number for
                                        // openMP
  convolution.convolve(*cloudout);

  return true;
}

/// 使用MLS进行平滑
bool MLSSmooth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudout,
               const double& radius) {
  if (cloud == nullptr) return false;
  if (cloud->points.size() < 10) return false;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_points_normal(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
  mls.setInputCloud(cloud);
  mls.setComputeNormals(false);  // 是否计算法线，设置为ture则计算法线
  mls.setPolynomialFit(
      false);  // 设置为true则在平滑过程中采用多项式拟合来提高精度
  mls.setPolynomialOrder(2);    // 设置MLS拟合的阶数，默认是2
  mls.setSearchMethod(tree);    // 邻域点搜索的方式
  mls.setSearchRadius(radius);  // 邻域搜索半径
  //   mls.setNumberOfThreads(10);   // 设置多线程加速的线程数
  mls.process(*cloudout);  // 曲面重建

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_lidar_merger_node");
  ros::NodeHandle nh;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile(argv[1], *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dn(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZ>());

  VoxelGridDownsample(cloud, cloud_dn, 0.025);
  // radius >= 0.2才行, 不然有洞
  // GaussianSmooth(cloud_dn, cloud_out, 0.2, 4, 0.1);
  MLSSmooth(cloud_dn, cloud_out, 0.05);

  pcl::io::savePCDFile(std::string(argv[1]) + "_smoothed.pcd", *cloud_out);
  pcl::visualization::CloudViewer viewer("fuck");

  while (ros::ok()) {
    viewer.showCloud(cloud_out, "test");
    sleep(1);
  }
}
