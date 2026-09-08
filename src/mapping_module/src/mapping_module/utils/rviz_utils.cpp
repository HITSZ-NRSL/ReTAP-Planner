/*
 * Created on Wed Dec 16 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: Ming Cao
 */

#include "utils/rviz_utils.h"

namespace rviz_utils {

visualization_msgs::Marker drawBoundingbox(const octomap::Boundingbox& bbox) {
  visualization_msgs::Marker bbox_msg;
  bbox_msg.ns = "lines";
  bbox_msg.action = visualization_msgs::Marker::MODIFY;
  bbox_msg.id = 2;
  bbox_msg.type = visualization_msgs::Marker::LINE_LIST;
  bbox_msg.scale.x = 0.1;
  bbox_msg.scale.y = 0.1;
  bbox_msg.scale.z = 0.1;
  bbox_msg.color.r = 1;
  bbox_msg.color.g = 0;
  bbox_msg.color.b = 0;
  bbox_msg.color.a = 1.0;

  if (bbox.isReset()) {
    return bbox_msg;
  }

  geometry_msgs::Point flu;  /* front left up */
  flu.x = bbox.maxX();
  flu.y = bbox.maxY();
  flu.z = bbox.maxZ();

  geometry_msgs::Point fru;  /* front right up */
  fru = flu;
  fru.y = bbox.minY();

  geometry_msgs::Point blu;  /* behind left up */
  blu = flu;
  blu.x = bbox.minX();

  geometry_msgs::Point bru;  /* behind left up */
  bru = blu;
  bru.y = bbox.minY();

  geometry_msgs::Point fld;  /* front left down */
  fld = flu;
  fld.z = bbox.minZ();

  geometry_msgs::Point frd;  /* front right down */
  frd = fld;
  frd.y = bbox.minY();

  geometry_msgs::Point bld;  /* behind left down */
  bld = fld;
  bld.x = bbox.minX();

  geometry_msgs::Point brd;  /* behind right down */
  brd = bld;
  brd.y = bbox.minY();

  std::vector<geometry_msgs::Point> points =
    {blu, flu, blu, bru, blu, bld, flu, fld, flu, fru, fru, bru, fru, frd,
     frd, brd, frd, fld, brd, bru, brd, bld, bld, fld};
  for (int i = 0; i < points.size(); ++i) {
    bbox_msg.points.push_back(points[i]);
  }
  return bbox_msg;
}

visualization_msgs::Marker convertBoundingboxMsg(const octomap::Boundingbox& bbox) {
  visualization_msgs::Marker bbox_msg;
  bbox_msg.ns = "points";
  bbox_msg.action = visualization_msgs::Marker::MODIFY;
  bbox_msg.id = 2;
  bbox_msg.type = visualization_msgs::Marker::POINTS;
  bbox_msg.scale.x = 0.1;
  bbox_msg.scale.y = 0.1;
  bbox_msg.scale.z = 0.1;
  bbox_msg.color.r = 1;
  bbox_msg.color.g = 0;
  bbox_msg.color.b = 0;
  bbox_msg.color.a = 1.0;

  if (bbox.isReset()) {
    return bbox_msg;
  }

  geometry_msgs::Point min_point;
  min_point.x = bbox.minX();
  min_point.y = bbox.minY();
  min_point.z = bbox.minZ();
  bbox_msg.points.push_back(min_point);

  geometry_msgs::Point max_point;
  max_point.x = bbox.maxX();
  max_point.y = bbox.maxY();
  max_point.z = bbox.maxZ();
  bbox_msg.points.push_back(max_point);

  return bbox_msg;
}


visualization_msgs::MarkerArray
displayScoresInPoses(const std::vector<geometry_msgs::Pose>& poses,
                         const std::vector<double>& scores) {
  visualization_msgs::MarkerArray result;
  for (int i = 0; i < poses.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.ns = "texts";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = poses[i];
    marker.id = i;
    marker.scale.z = 0.4;
    marker.color.a = 1;
    marker.color.r = 255;
    marker.color.g = 255;
    marker.color.b = 255;
    marker.text = std::to_string(scores[i]);
    result.markers.push_back(marker);
  }
  return result;
}
}  // namespace rviz_utils
