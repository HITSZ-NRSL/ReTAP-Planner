#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Eigen>
#include <vector>

class TrajectoryGeneratorWaypoint {
 private:
  double _qp_cost{};
  Eigen::MatrixXd _Q;
  Eigen::VectorXd _Px, _Py, _Pz;
  ros::Publisher _wp_traj_vis_pub, _wp_path_vis_pub;
  float _vis_traj_width = 0.15;
  float _poly_num1D;
  float _dev_order;
  float _min_order;
  float _Vel = 1;
  float _Acc = 1;
  std::vector<geometry_msgs::Point> _poly_waypoints;

 public:
  TrajectoryGeneratorWaypoint();

  ~TrajectoryGeneratorWaypoint();

  Eigen::MatrixXd PolyQPGeneration(int order, const Eigen::MatrixXd &Path,
                                   const Eigen::MatrixXd &Vel,
                                   const Eigen::MatrixXd &Acc,
                                   const Eigen::VectorXd &Time);
  void trajGeneration(std::vector<Eigen::Vector4d> wp_list);
  Eigen::VectorXd timeAllocation(Eigen::MatrixXd Path);
  static int Factorial(int x);
  Eigen::MatrixXd getCt(int n_seg, int d_order);
  Eigen::MatrixXd getM(int n_seg, int d_order, int p_num1d,
                       const Eigen::VectorXd &ts);
  Eigen::MatrixXd getQ(int n_seg, int d_order, int p_num1d,
                       const Eigen::VectorXd &ts);

  void visPolyWaypoints();
  void visWayPointPath(Eigen::MatrixXd path);
  void generatePolyWaypoints(Eigen::MatrixXd polyCoeff, Eigen::VectorXd time);
  Eigen::Vector4d getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t);
};

#endif
