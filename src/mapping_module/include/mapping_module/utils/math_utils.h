/*
 * Created on Tue Aug 10 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <utility>

namespace math_utils {
double angleWarp(double angle);

double angleDiff(double a, double b);

template <typename scalar>
Eigen::Matrix<scalar, 3, 3> quaternionToMatrix(scalar x, scalar y, scalar z,
                                               scalar w) {
  Eigen::Matrix<scalar, 3, 3> res;
  res(0, 0) = 1 - 2 * y * y - 2 * z * z;
  res(0, 1) = 2 * x * y - 2 * z * w;
  res(0, 2) = 2 * x * z + 2 * y * w;
  res(1, 0) = 2 * x * y + 2 * z * w;
  res(1, 1) = 1 - 2 * x * x - 2 * z * z;
  res(1, 2) = 2 * y * z - 2 * x * w;
  res(2, 0) = 2 * x * z - 2 * y * w;
  res(2, 1) = 2 * y * z + 2 * x * w;
  res(2, 2) = 1 - 2 * x * x - 2 * y * y;
  return res;
}

template <typename scalar>
scalar getYaw(Eigen::Matrix<scalar, 3, 3> rot) {
  scalar result = atan2(rot(1, 0), rot(0, 0));
  return result;
}

template <typename scalar>
Eigen::Matrix<scalar, 3, 1> getRPY(Eigen::Matrix<scalar, 3, 3> rot) {
  double pitch = asin(-rot(2, 0));
  double roll = asin(rot(2, 1) / cos(pitch));
  double yaw = math_utils::getYaw(rot);
  Eigen::Matrix<scalar, 3, 1> res;
  res(0) = roll;
  res(1) = pitch;
  res(2) = yaw;
  return res;
}

template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> addSE2(const Eigen::Matrix<Scalar, 3, 1> &lhs,
                                   const Eigen::Matrix<Scalar, 3, 1> &rhs) {
  Eigen::Matrix<Scalar, 3, 1> res;
  res.x() = lhs.x() + rhs.x();
  res.y() = lhs.y() + rhs.y();
  res.z() = angleWarp(lhs.z() + rhs.z());
  return res;
}

double sigmoid(double x);

double tanh(double x);

template <typename scalar>
Eigen::Transform<scalar, 3, Eigen::Isometry> stateVectorToTransformation(
    const Eigen::Matrix<scalar, 6, 1> &state) {
  Eigen::Transform<scalar, 3, Eigen::Isometry> res;
  res.translation().x() = state(0);
  res.translation().y() = state(1);
  res.translation().z() = state(2);
  res.matrix().block(0, 0, 3, 3) =
      (Eigen::AngleAxisd(state(5), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(state(4), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitX()))
          .matrix();
  return res;
}

template <typename scalar>
Eigen::Matrix<scalar, 6, 1> transformationToStateVector(
    const Eigen::Transform<scalar, 3, Eigen::Isometry> &trans) {
  Eigen::Matrix<scalar, 6, 1> res;
  res(0) = trans.translation().x();
  res(1) = trans.translation().y();
  res(2) = trans.translation().z();

  Eigen::Matrix<scalar, 3, 1> rpy = getRPY(trans.rotation());
  res(3) = rpy(0);
  res(4) = rpy(1);
  res(5) = rpy(2);

  return res;
}

template <typename scalar>
Eigen::Matrix<scalar, 6, 1> compose(
    const Eigen::Matrix<scalar, 6, 1> &state,
    const Eigen::Transform<scalar, 3, Eigen::Isometry> &trans) {
  Eigen::Transform<scalar, 3, Eigen::Isometry> state_transform =
      stateVectorToTransformation(state);
  Eigen::Transform<scalar, 3, Eigen::Isometry> composed_transform =
      state_transform * trans;

  Eigen::Matrix<scalar, 6, 1> res =
      transformationToStateVector(composed_transform);

  return res;
}

double rad2Degree(const double rad);
double degree2Rad(const double degree);

template <typename scalar>
bool isValidFloat(scalar value) {
  if (std::isnan(value) || std::isinf(value) || !std::isfinite(value))
    return false;
  return true;
}



}  // namespace math_utils

