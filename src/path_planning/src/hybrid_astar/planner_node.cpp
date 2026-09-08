#include <ros/ros.h>

#include <cstring>
#include <iostream>

#include "hybrid_astar/planner.h"
#define BACKWARD_HAS_BFD 1
#include <backward_ros/backward.hpp>
// possible movements
// R = 6, 6.75 DEG: 当R=6时，每个方向的变化量
// const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};
// const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};
// const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097};

// R = 3, 6.75 DEG
// const float Node3D::dy[] = { 0,        -0.0207946, 0.0207946};
// const float Node3D::dx[] = { 0.35342917352,   0.352612,  0.352612};
// const float Node3D::dt[] = { 0,         0.11780972451,   -0.11780972451};

// const float Node3D::dy[] = { 0,       -0.16578, 0.16578};
// const float Node3D::dx[] = { 1.41372, 1.40067, 1.40067};
// const float Node3D::dt[] = { 0,       0.2356194,   -0.2356194};

// L = 1, DEG = 45, R = 1.273
// const float Node3D::dy[] = { 0,  -0.37292,  -0.37292};
// const float Node3D::dx[] = { 1,   0.9003,    0.9003};
// const float Node3D::dt[] = { 0,   0.785,    -0.785};
// L = 0.5, DEG = 45, R = 0.637  // 角度太大，有的角度无法规划
// const float Node3D::dy[] = { 0,  -0.186462,   0.186462};
// const float Node3D::dx[] = { 0.5,   0.45016,    0.45016};
// const float Node3D::dt[] = { 0,   0.785,   -0.785};
// L = 1, DEG = 22.5, R = 2.546
// const float Node3D::dy[] = {0, -0.193839179, 0.193839179};
// const float Node3D::dx[] = {1, 0.974495358, 0.974495358};
// const float Node3D::dt[] = {0, 0.392699082, -0.392699082};
// L = 0.5, DEG = 22.5, R=1.273  // 不会走直线， dx改为1即可
// const float dy[] = {0, -0.09692,   0.09692};
// const float dx[] = {1, 0.487248,  0.487248};
// const float dt[] = {0, 0.392699, -0.392699};
// const float Node3D::dy[] = {0, -0.04846,   0.04846};
// const float Node3D::dx[] = {0.5, 0.243624,  0.243624};
// const float Node3D::dt[] = {0, 0.196349, -0.196349};
// L = 0.5, DEG = 10, R = 2.86
// const float Node3D::dy[] = {0, -0.04352,   0.04352};
// const float Node3D::dx[] = {0.5, 0.49747,  0.49747};
// const float Node3D::dt[] = {0, 0.392699, -0.392699};
const int HybridAStar::Node3D::dir = 3;
const float HybridAStar::Node3D::dx[3] = {1, 0.487248, 0.487248};
const float HybridAStar::Node3D::dy[3] = {0, -0.09692, 0.09692};
const float HybridAStar::Node3D::dt[3] = {0, 0.392699, -0.392699};

const int HybridAStar::Node2D::dir = 8;
const int HybridAStar::Node2D::dx[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
const int HybridAStar::Node2D::dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

int main(int argc, char** argv) {
  backward::SignalHandling sh;
  ros::init(argc, argv, "hybrid_star");

  HybridAStar::Planner hy;

  while (ros::ok()) {
    usleep(1000 * 10);
    ros::spinOnce();
    hy.plan();
  }
  return 0;
}
