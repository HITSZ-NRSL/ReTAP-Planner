/*
 * Created on Fri Dec 04 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: Ming Cao
 */

#include "mapping_module/utils/bounding_box.h"

#include <ros/ros.h>

#include <limits>

namespace octomap {

void Boundingbox::reset() {
  maxX() = std::numeric_limits<float>::lowest();
  maxY() = std::numeric_limits<float>::lowest();
  maxZ() = std::numeric_limits<float>::lowest();

  minX() = std::numeric_limits<float>::max();
  minY() = std::numeric_limits<float>::max();
  minZ() = std::numeric_limits<float>::max();

  reset_ = true;
}

Boundingbox Boundingbox::infinityMax() {
  Boundingbox bbox;
  bbox.maxX() = std::numeric_limits<float>::max();
  bbox.maxY() = std::numeric_limits<float>::max();
  bbox.maxZ() = std::numeric_limits<float>::max();

  bbox.minX() = std::numeric_limits<float>::lowest();
  bbox.minY() = std::numeric_limits<float>::lowest();
  bbox.minZ() = std::numeric_limits<float>::lowest();

  return bbox;
}

void Boundingbox::insertPoint(const octomap::point3d &pt) {
  minX() = std::min(pt.x(), minX());
  minY() = std::min(pt.y(), minY());
  minZ() = std::min(pt.z(), minZ());

  maxX() = std::max(pt.x(), maxX());
  maxY() = std::max(pt.y(), maxY());
  maxZ() = std::max(pt.z(), maxZ());
  reset_ = false;
}

void Boundingbox::insertPointCloud(const octomap::Pointcloud &pc) {
  for (int i = 0; i < pc.size(); ++i) {
    insertPoint(pc[i]);
  }
}

bool Boundingbox::ifContain(const octomap::point3d &pt) const {
  //   return (pt.x() >= minX() - 0.001 && pt.x() <= maxX() + 0.001 &&
  //           pt.y() >= minY() - 0.001 && pt.y() <= maxY() + 0.001 &&
  //           pt.z() >= minZ() - 0.001 && pt.z() <= maxZ() + 0.001);
  return (pt.x() >= minX() && pt.x() <= maxX() && pt.y() >= minY() &&
          pt.y() <= maxY() && pt.z() >= minZ() && pt.z() <= maxZ());
}

bool Boundingbox::ifContainApprox(const octomap::point3d &pt) const {
  return (pt.x() >= minX() - 0.001 && pt.x() <= maxX() + 0.001 &&
          pt.y() >= minY() - 0.001 && pt.y() <= maxY() + 0.001 &&
          pt.z() >= minZ() - 0.001 && pt.z() <= maxZ() + 0.001);
}

bool Boundingbox::ifIntersect(const Boundingbox &bbox) const {
  auto util_func = [](float min_1, float min_2, float max_1, float max_2) {
    bool res = min_1 <= min_2 && min_2 <= max_1;
    res = res || (min_1 <= max_2 && max_2 <= max_1);
    res = res || (min_2 <= max_1 && max_1 <= max_2);
    res = res || (min_2 <= min_1 && min_1 <= max_2);
    return res;
  };

  return util_func(minX(), bbox.minX(), maxX(), bbox.maxX()) &&
         util_func(minY(), bbox.minY(), maxY(), bbox.maxY()) &&
         util_func(minZ(), bbox.minZ(), maxZ(), bbox.maxZ());
}

float Boundingbox::computeIntersectArea(const Boundingbox &bbox) const {
  float intersect_x_min = std::max(this->minX(), bbox.minX());
  float intersect_x_max = std::min(this->maxX(), bbox.maxX());
  float intersect_y_min = std::max(this->minY(), bbox.minY());
  float intersect_y_max = std::min(this->maxY(), bbox.maxY());
  float intersect_z_min = std::max(this->minZ(), bbox.minZ());
  float intersect_z_max = std::min(this->maxZ(), bbox.maxZ());

  if (intersect_x_min >= intersect_x_max ||
      intersect_y_min >= intersect_y_max ||
      intersect_z_min >= intersect_z_max) {
    return 0.0f;
  }

  return (intersect_x_max - intersect_x_min) *
         (intersect_y_max - intersect_y_min) *
         (intersect_z_max - intersect_z_min);
}

Boundingbox Boundingbox::computeInteresctBbox(const Boundingbox &bbox) const {
  Boundingbox res;

  res.minX() = std::max(this->minX(), bbox.minX());
  res.maxX() = std::min(this->maxX(), bbox.maxX());
  res.minY() = std::max(this->minY(), bbox.minY());
  res.maxY() = std::min(this->maxY(), bbox.maxY());
  res.minZ() = std::max(this->minZ(), bbox.minZ());
  res.maxZ() = std::min(this->maxZ(), bbox.maxZ());

  return res;
}

float Boundingbox::area() const {
  if (isReset()) {
    return 0;
  }
  return (maxX() - minX()) * (maxY() - minY()) * (maxZ() - minZ());
}

std::ostream &operator<<(std::ostream &out, const Boundingbox &b) {
  out.precision(3);
  out << "Min: [" << b.minX() << ", " << b.minY() << ", " << b.minZ() << "], "
      << "Max: [" << b.maxX() << ", " << b.maxY() << ", " << b.maxZ() << "]. ";
  return out;
}

}  // namespace octomap
