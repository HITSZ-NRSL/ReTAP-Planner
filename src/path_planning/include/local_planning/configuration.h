#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>

class ConfigurationPoint {
 public:
  Eigen::Vector3f position;        // goal point
  Eigen::Quaternionf orientation;  // goal dir
  Eigen::Quaternionf pose;         // normal = pose * orientation
  Eigen::Vector3f offset;

  std::pair<float, float> flipper_angles;
  ros::WallDuration timecost;
  float acc_z;
};
