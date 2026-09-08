/* Copyright Year: 2023
 * Copyright Owner: Liyx
 */
#pragma once
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

class SampleSet {
 public:
  float track_width;
  float track_length;
  float track_separation;  // 左右两侧履带中心，的间距
  float flipper_length;
  float flipper_width;
  float flipper_separation;
  float resolution;
  float track_area;

  Eigen::Quaternionf pose;  // 底盘的yaw角
  pcl::PointCloud<pcl::PointXYZ> track_samples, front_flipper_samples,
      rear_flipper_samples, body_samples;
  pcl::PointCloud<pcl::PointXYZ> track_corners, body_corners,
      front_flipper_corners, rear_flipper_corners;  // for track visualization
  pcl::PointCloud<pcl::PointXYZ> base_border_corners;  // body + track + flipper

  SampleSet() {}
  SampleSet(float _track_width, float _track_length, float _track_separation,
            float _flipper_width, float _flipper_length,
            float _flipper_separation, float _resolution) {
    track_width = _track_width;    //
    track_length = _track_length;  //
    track_separation = _track_separation;
    flipper_length = _flipper_length;
    flipper_width = _flipper_width;
    flipper_separation = _flipper_separation;
    resolution = _resolution;
    track_area = (track_width + track_separation) * track_length;

    // fill the <SampleSet>initial_track_samples_ structure
    pose = Eigen::Quaternionf::Identity();

    // generate track samples
    float offset_x = 0;
    float offset_y = 0.5 * track_separation;
    for (float dx = 0; dx <= 0.5 * track_length; dx += resolution) {
      for (float dy = 0; dy <= 0.5 * track_width; dy += resolution) {
        // 1.1 sampling points for the right track(y)
        track_samples.push_back(pcl::PointXYZ(offset_x + dx, offset_y + dy, 0));
        track_samples.push_back(pcl::PointXYZ(offset_x + dx, offset_y - dy, 0));
        track_samples.push_back(pcl::PointXYZ(offset_x - dx, offset_y + dy, 0));
        track_samples.push_back(pcl::PointXYZ(offset_x - dx, offset_y - dy, 0));

        // 1.2 sampling points for the left track(-y)
        track_samples.push_back(
            pcl::PointXYZ(offset_x + dx, -offset_y + dy, 0));
        track_samples.push_back(
            pcl::PointXYZ(offset_x + dx, -offset_y - dy, 0));
        track_samples.push_back(
            pcl::PointXYZ(offset_x - dx, -offset_y + dy, 0));
        track_samples.push_back(
            pcl::PointXYZ(offset_x - dx, -offset_y - dy, 0));
      }
    }

    // generate front flipper samples
    offset_x = 0.5 * (track_length + flipper_length);
    offset_y = 0.5 * flipper_separation;
    for (float dx = 0; dx <= 0.5 * flipper_length; dx += resolution) {
      for (float dy = 0; dy < 0.5 * flipper_width; dy += resolution) {
        // 2.1 sampling points for the front right flipper(x)
        front_flipper_samples.push_back(
            pcl::PointXYZ(offset_x + dx, offset_y + dy, 0));
        front_flipper_samples.push_back(
            pcl::PointXYZ(offset_x + dx, offset_y - dy, 0));
        front_flipper_samples.push_back(
            pcl::PointXYZ(offset_x - dx, offset_y + dy, 0));
        front_flipper_samples.push_back(
            pcl::PointXYZ(offset_x - dx, offset_y - dy, 0));

        // 2.2 sampling points for the front left flipper(x)
        front_flipper_samples.push_back(
            pcl::PointXYZ(offset_x + dx, -offset_y + dy, 0));
        front_flipper_samples.push_back(
            pcl::PointXYZ(offset_x + dx, -offset_y - dy, 0));
        front_flipper_samples.push_back(
            pcl::PointXYZ(offset_x - dx, -offset_y + dy, 0));
        front_flipper_samples.push_back(
            pcl::PointXYZ(offset_x - dx, -offset_y - dy, 0));
      }
    }

    // generate rear flipper samples
    offset_x = -0.5 * (track_length + flipper_length);
    offset_y = 0.5 * flipper_separation;
    for (float dx = 0; dx <= 0.5 * flipper_length; dx += resolution) {
      for (float dy = 0; dy < 0.5 * flipper_width; dy += resolution) {
        // 2.3 sampling points for the front right flipper(x)
        rear_flipper_samples.push_back(
            pcl::PointXYZ(offset_x + dx, offset_y + dy, 0));
        rear_flipper_samples.push_back(
            pcl::PointXYZ(offset_x + dx, offset_y - dy, 0));
        rear_flipper_samples.push_back(
            pcl::PointXYZ(offset_x - dx, offset_y + dy, 0));
        rear_flipper_samples.push_back(
            pcl::PointXYZ(offset_x - dx, offset_y - dy, 0));

        // 2.4 sampling points for the front left flipper(x)
        rear_flipper_samples.push_back(
            pcl::PointXYZ(offset_x + dx, -offset_y + dy, 0));
        rear_flipper_samples.push_back(
            pcl::PointXYZ(offset_x + dx, -offset_y - dy, 0));
        rear_flipper_samples.push_back(
            pcl::PointXYZ(offset_x - dx, -offset_y + dy, 0));
        rear_flipper_samples.push_back(
            pcl::PointXYZ(offset_x - dx, -offset_y - dy, 0));
      }
    }

    // fill the track corners for visualization
    {
      // left track corners: front(left,right), rear(right,left)
      track_corners.push_back(pcl::PointXYZ(
          0.5 * track_length, -0.5 * (track_width + track_separation), 0));
      track_corners.push_back(pcl::PointXYZ(
          0.5 * track_length, -0.5 * (track_separation - track_width), 0));
      track_corners.push_back(pcl::PointXYZ(
          -0.5 * track_length, -0.5 * (track_separation - track_width), 0));
      track_corners.push_back(pcl::PointXYZ(
          -0.5 * track_length, -0.5 * (track_width + track_separation), 0));

      // right track corners: front(left,right), rear(right, left)
      track_corners.push_back(pcl::PointXYZ(
          0.5 * track_length, 0.5 * (track_separation - track_width), 0));
      track_corners.push_back(pcl::PointXYZ(
          0.5 * track_length, 0.5 * (track_width + track_separation), 0));
      track_corners.push_back(pcl::PointXYZ(
          -0.5 * track_length, 0.5 * (track_width + track_separation), 0));
      track_corners.push_back(pcl::PointXYZ(
          -0.5 * track_length, 0.5 * (track_separation - track_width), 0));
    }

    // fill the front flipper corners for visualization
    {
      // left front flipper corners: front(left,right), rear(right,left)
      front_flipper_corners.push_back(pcl::PointXYZ(
          0.5 * track_length, -0.5 * (flipper_separation + flipper_width), 0));
      front_flipper_corners.push_back(pcl::PointXYZ(
          0.5 * track_length, -0.5 * (flipper_separation - flipper_width), 0));
      front_flipper_corners.push_back(
          pcl::PointXYZ(0.5 * track_length + flipper_length,
                        -0.5 * (flipper_separation - flipper_width), 0));
      front_flipper_corners.push_back(
          pcl::PointXYZ(0.5 * track_length + flipper_length,
                        -0.5 * (flipper_separation + flipper_width), 0));

      // right front flipper corners: front(left,right), rear(right, left)
      front_flipper_corners.push_back(pcl::PointXYZ(
          0.5 * track_length, 0.5 * (flipper_separation - flipper_width), 0));
      front_flipper_corners.push_back(pcl::PointXYZ(
          0.5 * track_length, 0.5 * (flipper_separation + flipper_width), 0));
      front_flipper_corners.push_back(
          pcl::PointXYZ(0.5 * track_length + flipper_length,
                        0.5 * (flipper_separation + flipper_width), 0));
      front_flipper_corners.push_back(
          pcl::PointXYZ(0.5 * track_length + flipper_length,
                        0.5 * (flipper_separation - flipper_width), 0));
    }

    // fill the rear flipper corners for visualization
    {
      // left rear flipper corners: front(left,right), rear(right,left)
      rear_flipper_corners.push_back(pcl::PointXYZ(
          -0.5 * track_length, -0.5 * (flipper_separation + flipper_width), 0));
      rear_flipper_corners.push_back(pcl::PointXYZ(
          -0.5 * track_length, -0.5 * (flipper_separation - flipper_width), 0));
      rear_flipper_corners.push_back(
          pcl::PointXYZ(-0.5 * track_length + flipper_length,
                        -0.5 * (flipper_separation - flipper_width), 0));
      rear_flipper_corners.push_back(
          pcl::PointXYZ(-0.5 * track_length + flipper_length,
                        -0.5 * (flipper_separation + flipper_width), 0));

      // right rear flipper corners: front(left,right), rear(right, left)
      rear_flipper_corners.push_back(pcl::PointXYZ(
          -0.5 * track_length, 0.5 * (flipper_separation - flipper_width), 0));
      rear_flipper_corners.push_back(pcl::PointXYZ(
          -0.5 * track_length, 0.5 * (flipper_separation + flipper_width), 0));
      rear_flipper_corners.push_back(
          pcl::PointXYZ(-0.5 * track_length + flipper_length,
                        0.5 * (flipper_separation + flipper_width), 0));
      rear_flipper_corners.push_back(
          pcl::PointXYZ(-0.5 * track_length + flipper_length,
                        0.5 * (flipper_separation - flipper_width), 0));
    }

    // fill the base border corners in_clockwise for points filtering
    {
      // front right
      base_border_corners.push_back(
          pcl::PointXYZ(0.5 * track_length + flipper_length,
                        0.5 * (flipper_width + flipper_separation), 0));
      // front left
      base_border_corners.push_back(
          pcl::PointXYZ(0.5 * track_length + flipper_length,
                        -0.5 * (flipper_width + flipper_separation), 0));
      // rear left
      base_border_corners.push_back(
          pcl::PointXYZ(-0.5 * track_length + flipper_length,
                        -0.5 * (flipper_width + flipper_separation), 0));
      // rear right
      base_border_corners.push_back(
          pcl::PointXYZ(-0.5 * track_length + flipper_length,
                        0.5 * (flipper_width + flipper_separation), 0));
    }

    // generate body_samples for collision check
    offset_x = 0;
    offset_y = 0;
    float body_width = track_separation - track_width;
    for (float dx = 0; dx <= 0.5 * track_length; dx += resolution) {
      for (float dy = 0; dy < 0.5 * body_width - resolution; dy += resolution) {
        body_samples.push_back(pcl::PointXYZ(offset_x + dx, offset_y + dy, 0));
        body_samples.push_back(pcl::PointXYZ(offset_x + dx, offset_y - dy, 0));
        body_samples.push_back(pcl::PointXYZ(offset_x - dx, offset_y + dy, 0));
        body_samples.push_back(pcl::PointXYZ(offset_x - dx, offset_y - dy, 0));
      }
    }

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    for (int i = 0; i < track_samples.size(); i++) {
      pcl::PointXYZRGB p(0, 255, 0);
      p.x = track_samples.points[i].x;
      p.y = track_samples.points[i].y;
      p.z = 0;
      cloud.push_back(p);
    }
    for (int i = 0; i < body_samples.size(); i++) {
      pcl::PointXYZRGB p(255, 0, 0);
      p.x = body_samples.points[i].x;
      p.y = body_samples.points[i].y;
      p.z = 0;
      cloud.push_back(p);
    }
    for (int i = 0; i < front_flipper_samples.size(); i++) {
      pcl::PointXYZRGB p(0, 0, 255);
      p.x = front_flipper_samples.points[i].x;
      p.y = front_flipper_samples.points[i].y;
      p.z = 0;
      cloud.push_back(p);
    }
    for (int i = 0; i < rear_flipper_samples.size(); i++) {
      pcl::PointXYZRGB p(0, 0, 255);
      p.x = rear_flipper_samples.points[i].x;
      p.y = rear_flipper_samples.points[i].y;
      p.z = 0;
      cloud.push_back(p);
    }
    // pcl::io::savePCDFile("/media/D_DOC/samples.pcd", cloud);
  }

  SampleSet(Eigen::Quaternionf _pose, SampleSet _other) {
    pose = _pose;
    track_width = _other.track_width;    //
    track_length = _other.track_length;  //
    track_separation = _other.track_separation;
    flipper_length = _other.flipper_length;
    flipper_width = _other.flipper_width;
    flipper_separation = _other.flipper_separation;
    resolution = _other.resolution;
    track_area = _other.track_area;

    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.rotate(pose);

    pcl::transformPointCloud(_other.track_samples, track_samples, T.matrix());
    pcl::transformPointCloud(_other.front_flipper_samples,
                             front_flipper_samples, T.matrix());
    pcl::transformPointCloud(_other.rear_flipper_samples, rear_flipper_samples,
                             T.matrix());

    pcl::transformPointCloud(_other.body_samples, body_samples, T.matrix());
    pcl::transformPointCloud(_other.track_corners, track_corners, T.matrix());
    pcl::transformPointCloud(_other.body_corners, body_corners, T.matrix());
    pcl::transformPointCloud(_other.front_flipper_corners,
                             front_flipper_corners, T.matrix());
    pcl::transformPointCloud(_other.rear_flipper_corners, rear_flipper_corners,
                             T.matrix());
    pcl::transformPointCloud(_other.base_border_corners, base_border_corners,
                             T.matrix());
  }
};
