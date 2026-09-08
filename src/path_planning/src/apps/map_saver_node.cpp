#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "mapping_module/world_representation/world_representation.h"

ros::Subscriber octomap_subscriber_;
std::shared_ptr<octomap::IgTree> octree_ptr_;
pcl::PointCloud<pcl::PointXYZ> pcd_;
std::string base_footprint_frame_ = "base_link";  // NOLINT
std::string map_frame_ = "map";                   // NOLINT
std::string data_folder_;
bool map_flag = false;
void OctomapCallback(const octomap_msgs::Octomap &msg) {
  octomap::IgTree temp_ig_tree(0.5);
  octomap::AbstractOcTree *aot = octomap_msgs::msgToMap(msg);
  if (aot) {
    if (!octree_ptr_) octree_ptr_.reset(new octomap::IgTree(0.2));
    octree_ptr_.reset(dynamic_cast<octomap::IgTree *>(aot));
    // 转为点云
    // pcd_.clear();
    // auto start = octree_ptr_->begin_leafs();
    // auto end = octree_ptr_->end_leafs();
    // for (auto iter = start; iter != end; iter++) {
    //   auto p3d = octree_ptr_->keyToCoord(iter.getKey());
    //   pcd_.push_back(pcl::PointXYZ(p3d.x(), p3d.y(), p3d.z()));
    // }
    map_flag = true;
  }
}

bool getSensorPoseTF(tf::StampedTransform *pose, ros::Time timestamp) {
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

bool getSensorPoseEigen(Eigen::Isometry3d *pose, ros::Time timestamp) {
  tf::StampedTransform lidar_transform_tf;
  if (!getSensorPoseTF(&lidar_transform_tf, timestamp)) {
    return false;
  }
  tf::transformTFToEigen(lidar_transform_tf, *pose);

  return true;
}

void writeOneRecordToFile(std::string file_name, Eigen::Isometry3d base_pose) {
  std::string file_path = data_folder_ + "/record.txt";
  std::ofstream out(file_path, std::ios::app);  // 追加写入
  Eigen::Vector3d p = base_pose.translation();
  Eigen::Quaterniond q(base_pose.rotation());
  std::stringstream ss;
  ss << file_name << "\t" << p.x() << "\t" << p.y() << "\t" << p.z() << "\t"
     << q.x() << "\t" << q.y() << "\t" << q.z() << "\t" << q.w() << "\n";
  if (!out.fail()) {
    out << ss.str();
  }
  out.close();
  std::cout << "<writeOneRecordToFile>: " << ss.str();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_saver_node");
  ros::NodeHandle nh;
  octomap_subscriber_ =
      nh.subscribe("mapping_module/local_octree", 1, &OctomapCallback);

  if (argc != 2) {
    ROS_ERROR("No input folder path! argc=%d", argc);
    return 0;
  } else {
    data_folder_ = std::string(argv[argc - 1]);
    std::cout << "data_folder_: " << data_folder_ << std::endl;
  }
  std::string file_path = data_folder_ + "/record.txt";
  Eigen::Vector3d last_position(0, 0, 0);
  while (ros::ok()) {
    if (map_flag) {
      map_flag = false;
      Eigen::Isometry3d base_pose_;
      if (getSensorPoseEigen(&base_pose_, ros::Time(0))) {
        // 判断两次的位置变化
        Eigen::Vector3d position = base_pose_.translation();
        if ((position - last_position).norm() > 2) {
          last_position = position;
          // 保存地图，同时保存位姿到txt文档
          std::stringstream ss;
          ss.precision(20);
          ss << data_folder_ << "/" << ros::WallTime::now().toSec() << ".bt";
          // pcl::io::savePCDFile(ss.str(), pcd_);
          octree_ptr_->write(ss.str());
          writeOneRecordToFile(ss.str(), base_pose_);
        }
      }
    }
    ros::spinOnce();
    usleep(1e5);
  }
}
