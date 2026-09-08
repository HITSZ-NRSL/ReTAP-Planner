/*
 * Created on Wed Dec 16 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include <octomap/ColorOcTree.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <iostream>
#define BACKWARD_HAS_BFD 1
#include <backward_ros/backward.hpp>
#include "mapping_module/mapping_module.h"

using OcTreeType = octomap::IgTree;

int main(int argc, char **argv) {
  backward::SignalHandling sh;
  ros::init(argc, argv, "mapping_module_node");
  ros::NodeHandle nh("mapping_module_node");

  mapping_module::MappingModule mapping_module_;
  ros::MultiThreadedSpinner spinner(8);
  spinner.spin();

}
