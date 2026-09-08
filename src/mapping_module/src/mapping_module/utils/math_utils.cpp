/*
 * Created on Tue Aug 24 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: Ming Cao
 */

#include "mapping_module/utils/math_utils.h"

double math_utils::angleWarp(double angle) {
  angle = fmod(angle + M_PI, 2 * M_PI);
  if (angle < 0) {
    angle += 2 * M_PI;
  }
  return angle - M_PI;
}

double math_utils::angleDiff(double a, double b) {
  // double diff = fmod(a - b + M_PI, 2 * M_PI);
  // if (diff < 0) {
  //   diff += M_PI * 2;
  // }
  // return diff - M_PI;
  return angleWarp(a - b);
}

double math_utils::sigmoid(double x) { return 1 / (1 + std::exp(-x)); }

double math_utils::tanh(double x) { return 2 / (1 + std::exp(-2 * x)) - 1; }

double math_utils::rad2Degree(const double rad) {
  return rad / M_PI * 180;
}

double math_utils::degree2Rad(const double degree) {
  return degree / 180 * M_PI;
}
