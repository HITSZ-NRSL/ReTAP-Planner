/*
 * Created on Fri Dec 04 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#pragma once

#include <octomap/Pointcloud.h>
#include <octomap/octomap_types.h>
#include <pcl/point_cloud.h>

#include <algorithm>
#include <utility>

namespace octomap {
struct Boundingbox {
  octomap::point3d bottom_right_front_pt;
  octomap::point3d top_left_rear_pt;

  Boundingbox() { reset(); }
  Boundingbox(float min_x, float min_y, float min_z, float max_x, float max_y,
              float max_z) {
    minX() = min_x;
    minY() = min_y;
    minZ() = min_z;
    maxX() = max_x;
    maxY() = max_y;
    maxZ() = max_z;
  }

  void reset();

  void insertPoint(const octomap::point3d &pt);

  template <typename pointT>
  void insertPoint(const pointT &t) {
    minX() = std::min(t.x, minX());
    minY() = std::min(t.y, minY());
    minZ() = std::min(t.z, minZ());

    maxX() = std::max(t.x, maxX());
    maxY() = std::max(t.y, maxY());
    maxZ() = std::max(t.z, maxZ());
    reset_ = false;
  }

  template <typename pointT>
  void insertPointCloud(const pcl::PointCloud<pointT> &pc) {
    for (int i = 0; i < pc.size(); ++i) {
      insertPoint(pc[i]);
    }
  }

  template <typename pointT>
  pcl::PointCloud<pointT> filterPointCloud(const pcl::PointCloud<pointT> &pc,
                                           bool inverse = false) const {
    pcl::PointCloud<pointT> res;
    for (int i = 0; i < pc.size(); ++i) {
      bool contained = ifContain(pc[i]);
      if ((contained && !inverse) || (!contained && inverse)) {
        res.push_back(pc[i]);
      }
    }
    return res;
  }

  void insertPointCloud(const octomap::Pointcloud &pc);

  bool isReset() const { return reset_; }

  bool ifContain(const octomap::point3d &pt) const;

  bool ifContainApprox(const octomap::point3d &pt) const;

  template <typename pointT>
  bool ifContain(const pointT &pt) const {
    octomap::point3d cur_pt(pt.x, pt.y, pt.z);
    return ifContain(cur_pt);
  }

  bool ifIntersect(const Boundingbox &bbox) const;

  float computeIntersectArea(const Boundingbox &bbox) const;

  Boundingbox computeInteresctBbox(const Boundingbox &bbox) const;

  float area() const;

  float maxX() const { return top_left_rear_pt.x(); }
  float &maxX() { return top_left_rear_pt.x(); }

  float maxY() const { return top_left_rear_pt.y(); }
  float &maxY() { return top_left_rear_pt.y(); }

  float maxZ() const { return top_left_rear_pt.z(); }
  float &maxZ() { return top_left_rear_pt.z(); }

  float minX() const { return bottom_right_front_pt.x(); }
  float &minX() { return bottom_right_front_pt.x(); }

  float minY() const { return bottom_right_front_pt.y(); }
  float &minY() { return bottom_right_front_pt.y(); }

  float minZ() const { return bottom_right_front_pt.z(); }
  float &minZ() { return bottom_right_front_pt.z(); }

  octomap::point3d center() {
    return octomap::point3d((maxPoint() + minPoint()) * 0.5);
  }
  octomap::point3d size() { return octomap::point3d(maxPoint() - minPoint()); }

  octomap::point3d minPoint() const {
    return octomap::point3d(minX(), minY(), minZ());
  }

  octomap::point3d maxPoint() const {
    return octomap::point3d(maxX(), maxY(), maxZ());
  }

  static Boundingbox infinityMax();

  void enlarge(float delta) {
    minX() = minX() - delta;
    minY() = minY() - delta;
    minZ() = minZ() - delta;

    maxX() = maxX() + delta;
    maxY() = maxY() + delta;
    maxZ() = maxZ() + delta;
  }

  friend std::ostream &operator<<(std::ostream &out, const Boundingbox &b);

 private:
  bool reset_ = true;
};
}  // namespace octomap
