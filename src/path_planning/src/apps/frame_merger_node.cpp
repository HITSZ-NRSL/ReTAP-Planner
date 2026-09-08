#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <mutex>
#include <thread>

ros::Publisher global_pcd_pub_;

std::string lidar_topic = "/os1_node/points";  // NOLINT
std::string world_frame_ = "world";           // NOLINT
std::string lidar_frame_ = "ouster_lidar";    // NOLINT

std::string file_path = "global_map.pcd";  // NOLINT
float resolution = 0.025;

pcl::PointCloud<pcl::PointXYZ>::Ptr global_pc_pcl;
std::mutex update_mutex_;

void mergePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr local_pc_pcl,
              Eigen::Isometry3d trans) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_pc_pcl2(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr world_pc_pcl(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (int i = 0; i < local_pc_pcl->size(); i++) {
    Eigen::Vector3f p(local_pc_pcl->at(i).x, local_pc_pcl->at(i).y,
                      local_pc_pcl->at(i).z);
    float dis = p.norm();
    if (dis > 10 || dis < 1.5) continue;
    local_pc_pcl2->push_back(local_pc_pcl->at(i));
  }

  pcl::transformPointCloud(*local_pc_pcl2, *world_pc_pcl, trans.matrix());
  *global_pc_pcl += *world_pc_pcl;
}

// 订阅lidar和tf，合并点云到global_pc_pcl
void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &lidar_msg) {
  tf::StampedTransform lidar_transform_tf;
  static tf::TransformListener listener;
  const double lidar_tf_dt_ = 0;
  ros::Time lidar_stamp = lidar_msg->header.stamp + ros::Duration(lidar_tf_dt_);
  try {
    listener.waitForTransform(world_frame_, lidar_frame_, lidar_stamp,
                              ros::Duration(0.3));
    listener.lookupTransform(world_frame_, lidar_frame_, lidar_stamp,
                             lidar_transform_tf);
  } catch (...) {
    std::cout << "Sensor data is earlier than tf! world_frame: " << world_frame_
              << ", lidar_frame_: " << lidar_frame_ << std::endl;
    return;
  }

  Eigen::Isometry3d lidar_transform_eigen = Eigen::Isometry3d::Identity();
  tf::transformTFToEigen(lidar_transform_tf, lidar_transform_eigen);
  {
    std::lock_guard<std::mutex> locker(update_mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_pc_pcl(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*lidar_msg, *local_pc_pcl);
    mergePCD(local_pc_pcl, lidar_transform_eigen);
  }
}

// 下采样，存盘并发布
void run() {
  ros::Rate rate(5);
  while (ros::ok()) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_pc_pcl(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_pc_pcl(
        new pcl::PointCloud<pcl::PointXYZ>);
    {
      std::lock_guard<std::mutex> locker(update_mutex_);
      *world_pc_pcl = *global_pc_pcl;
    }

    pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
    vg_filter.setInputCloud(world_pc_pcl);
    vg_filter.setLeafSize(resolution, resolution, resolution);
    vg_filter.filter(*filter_pc_pcl);
    *world_pc_pcl = *filter_pc_pcl;

    world_pc_pcl->height = 1;
    world_pc_pcl->width = world_pc_pcl->points.size();
    if (world_pc_pcl->width == 0) {
      std::cout << "<run>: world_pc_pcl is NULL. " << std::endl;
      rate.sleep();
      continue;
    }
    world_pc_pcl->is_dense = false;

    sensor_msgs::PointCloud2 lidar_msg2;
    pcl::toROSMsg(*world_pc_pcl, lidar_msg2);
    std::cout << "<run>: publish. " << std::endl;
    lidar_msg2.header.frame_id = world_frame_;
    lidar_msg2.header.stamp = ros::Time::now();
    global_pcd_pub_.publish(lidar_msg2);
    pcl::io::savePCDFile(file_path, *world_pc_pcl, true);
    rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "frame_merger_node");
  ros::NodeHandle nh_;
  nh_.param("/frame_merger_node/file_path", file_path, file_path);
  std::cout << "<frame_merger_node>: save file_path = " << file_path << std::endl;

  global_pc_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>);

  ros::Subscriber lidar_sub_ = nh_.subscribe(lidar_topic, 1, &lidarCallback);
  global_pcd_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("laser_cloud_map", 1);

  std::thread run_thread_ = std::thread(std::bind(&run));
  ros::spin();
}