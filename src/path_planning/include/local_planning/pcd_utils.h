#include "local_planning/local_planning.h"

void LocalPlanning::LoadOctomapFromPCD(std::string input_file) {
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::io::loadPCDFile<pcl::PointXYZRGBA>(input_file, cloud);

  std::lock_guard<std::mutex> lock_octo(local_octomap_mutex_);
  local_octree_ptr_.reset(new octomap::IgTree(0.025));
  for (auto p : cloud.points) {
    local_octree_ptr_->updateNode(octomap::point3d(p.x, p.y, p.z), true);
  }
  local_octree_ptr_->updateInnerOccupancy();
}

void LocalPlanning::LoadOctomapFromBT(std::string input_file) {  // NOLINT

  std::lock_guard<std::mutex> lock_octo(local_octomap_mutex_);
  if (!local_octree_ptr_) local_octree_ptr_.reset(new octomap::IgTree(0.025));
  local_octree_ptr_.reset(
      dynamic_cast<octomap::IgTree *>(local_octree_ptr_->read(input_file)));
}

void LocalPlanning::ColorPointcloud(
    pcl::PointCloud<pcl::PointXYZ> &cloud_in, int r, int g, int b,
    pcl::PointCloud<pcl::PointXYZRGB> &cloud_out) {
  for (auto p : cloud_in) {
    pcl::PointXYZRGB pc(r, g, b);
    pc.x = p.x;
    pc.y = p.y;
    pc.z = p.z;
    cloud_out.push_back(pc);
  }
}

void LocalPlanning::DecolorPointcloud(
    pcl::PointCloud<pcl::PointXYZRGB> &cloud_in,
    pcl::PointCloud<pcl::PointXYZ> &cloud_out) {
  for (auto p : cloud_in) {
    cloud_out.push_back(pcl::PointXYZ(p.x, p.y, p.z));
  }
}

void LocalPlanning::PublishConvexHullMarkers(
    pcl::PointCloud<pcl::PointXYZRGB> &hull_pcd) {
  visualization_msgs::Marker bbox_msg;
  bbox_msg.ns = "convex";
  bbox_msg.header.frame_id = map_frame_;
  bbox_msg.header.stamp = ros::Time::now();
  // bbox_msg.action = visualization_msgs::Marker::DELETEALL;
  // track_convex_hull_publisher_.publish(bbox_msg);

  bbox_msg.header.stamp = ros::Time::now();
  bbox_msg.action = visualization_msgs::Marker::ADD;
  bbox_msg.id = 1;
  bbox_msg.type = visualization_msgs::Marker::LINE_LIST;
  bbox_msg.scale.x = 0.02;
  bbox_msg.scale.y = 0.02;
  bbox_msg.scale.z = 0.02;
  bbox_msg.color.r = 0.5;
  bbox_msg.color.g = 0;
  bbox_msg.color.b = 0.5;
  bbox_msg.color.a = 1.0;
  bbox_msg.pose.orientation.w = 1;
  bbox_msg.pose.orientation.x = 0;
  bbox_msg.pose.orientation.y = 0;
  bbox_msg.pose.orientation.z = 0;
  bbox_msg.pose.position.x = 0;
  bbox_msg.pose.position.y = 0;
  bbox_msg.pose.position.z = 0;

  geometry_msgs::Point pt;
  for (int i = 0; i < hull_pcd.size(); i++) {
    auto p0 = hull_pcd.points[i];
    pt.x = p0.x, pt.y = p0.y, pt.z = p0.z;
    bbox_msg.points.push_back(pt);
    auto p1 = hull_pcd.points[(i + 1) % hull_pcd.size()];
    pt.x = p1.x, pt.y = p1.y, pt.z = p1.z;
    bbox_msg.points.push_back(pt);
  }

  if (bbox_msg.points.size() > 0)
    track_convex_hull_publisher_.publish(bbox_msg);
}

void LocalPlanning::PublishTrackBoundaryMarkers(
    pcl::PointCloud<pcl::PointXYZRGB> &boundaries_pcd) {
  std::lock_guard<std::mutex> lock(track_boundary_publisher_mutex_);
  visualization_msgs::Marker bbox_msg;
  bbox_msg.ns = "lines";
  bbox_msg.header.frame_id = map_frame_;
  // bbox_msg.action = visualization_msgs::Marker::DELETEALL;
  // track_boundry_publisher.publish(bbox_msg);

  bbox_msg.action = visualization_msgs::Marker::ADD;
  bbox_msg.id = 1;
  bbox_msg.type = visualization_msgs::Marker::LINE_LIST;
  bbox_msg.scale.x = 0.02;
  bbox_msg.scale.y = 0.02;
  bbox_msg.scale.z = 0.02;
  bbox_msg.color.r = 1;
  bbox_msg.color.g = 1;
  bbox_msg.color.b = 0;
  bbox_msg.color.a = 1.0;

  geometry_msgs::Point pt;
  for (int i = 0; i < 4; i++) {
    auto p0 = boundaries_pcd.points[i];
    geometry_msgs::Point pt;
    pt.x = p0.x;
    pt.y = p0.y;
    pt.z = p0.z;
    bbox_msg.points.push_back(pt);
    auto p1 = boundaries_pcd.points[(i + 1) % 4];
    pt.x = p1.x;
    pt.y = p1.y;
    pt.z = p1.z;
    bbox_msg.points.push_back(pt);
  }

  for (int i = 4; i < 8; i++) {
    auto p0 = boundaries_pcd.points[i];
    geometry_msgs::Point pt;
    pt.x = p0.x;
    pt.y = p0.y;
    pt.z = p0.z;
    bbox_msg.points.push_back(pt);
    auto p1 = boundaries_pcd.points[(i + 1) % 4 + 4];
    pt.x = p1.x;
    pt.y = p1.y;
    pt.z = p1.z;
    bbox_msg.points.push_back(pt);
  }
  if (bbox_msg.points.size() > 0) track_boundry_publisher.publish(bbox_msg);
}

void LocalPlanning::PublishFlipperBoundaryMarkers(
    pcl::PointCloud<pcl::PointXYZRGB> &boundaries_pcd) {
  std::lock_guard<std::mutex> lock(flipper_boundary_publisher_mutex_);
  visualization_msgs::Marker bbox_msg;
  bbox_msg.ns = "lines";
  bbox_msg.header.frame_id = map_frame_;
  // bbox_msg.action = visualization_msgs::Marker::DELETEALL;
  // flipper_boundry_publisher.publish(bbox_msg);

  bbox_msg.action = visualization_msgs::Marker::ADD;
  bbox_msg.id = 1;
  bbox_msg.type = visualization_msgs::Marker::LINE_LIST;
  bbox_msg.scale.x = 0.02;
  bbox_msg.scale.y = 0.02;
  bbox_msg.scale.z = 0.02;
  bbox_msg.color.r = 1;
  bbox_msg.color.g = 1;
  bbox_msg.color.b = 0;
  bbox_msg.color.a = 1.0;

  geometry_msgs::Point pt;
  for (int i = 0; i < 4; i++) {
    auto p0 = boundaries_pcd.points[i];
    geometry_msgs::Point pt;
    pt.x = p0.x;
    pt.y = p0.y;
    pt.z = p0.z;
    bbox_msg.points.push_back(pt);
    auto p1 = boundaries_pcd.points[(i + 1) % 4];
    pt.x = p1.x;
    pt.y = p1.y;
    pt.z = p1.z;
    bbox_msg.points.push_back(pt);
  }

  for (int i = 4; i < 8; i++) {
    auto p0 = boundaries_pcd.points[i];
    geometry_msgs::Point pt;
    pt.x = p0.x;
    pt.y = p0.y;
    pt.z = p0.z;
    bbox_msg.points.push_back(pt);
    auto p1 = boundaries_pcd.points[(i + 1) % 4 + 4];
    pt.x = p1.x;
    pt.y = p1.y;
    pt.z = p1.z;
    bbox_msg.points.push_back(pt);
  }

  if (boundaries_pcd.size() > 8) {
    for (int i = 8; i < 12; i++) {
      auto p0 = boundaries_pcd.points[i];
      geometry_msgs::Point pt;
      pt.x = p0.x;
      pt.y = p0.y;
      pt.z = p0.z;
      bbox_msg.points.push_back(pt);
      auto p1 = boundaries_pcd.points[(i + 1) % 4 + 8];
      pt.x = p1.x;
      pt.y = p1.y;
      pt.z = p1.z;
      bbox_msg.points.push_back(pt);
    }

    for (int i = 12; i < 16; i++) {
      auto p0 = boundaries_pcd.points[i];
      geometry_msgs::Point pt;
      pt.x = p0.x;
      pt.y = p0.y;
      pt.z = p0.z;
      bbox_msg.points.push_back(pt);
      auto p1 = boundaries_pcd.points[(i + 1) % 4 + 12];
      pt.x = p1.x;
      pt.y = p1.y;
      pt.z = p1.z;
      bbox_msg.points.push_back(pt);
    }
  }

  if (bbox_msg.points.size() > 0) flipper_boundry_publisher.publish(bbox_msg);
}

void LocalPlanning::ClearConvexHullMarkers() {
  visualization_msgs::Marker bbox_msg, bbox_msg2;
  bbox_msg.ns = "convex";
  bbox_msg.header.frame_id = map_frame_;
  bbox_msg.header.stamp = ros::Time::now();
  // bbox_msg.action = visualization_msgs::Marker::DELETEALL;
  // track_convex_hull_publisher_.publish(bbox_msg);
}

void LocalPlanning::PublishConvexHullMarkers(
    ros::Publisher &pub, pcl::PointCloud<pcl::PointXYZRGB> &hull_pcd,
    Eigen::Vector3f track_center) {
  std::lock_guard<std::mutex> lock(convex_hull_publisher_mutex_);
  visualization_msgs::Marker bbox_msg, bbox_msg2;
  bbox_msg.ns = "convex";
  bbox_msg.header.frame_id = map_frame_;
  bbox_msg.header.stamp = ros::Time::now();
  // bbox_msg.action = visualization_msgs::Marker::DELETEALL;
  // pub.publish(bbox_msg);
  if (hull_pcd.size() == 0) return;

  bbox_msg.action = visualization_msgs::Marker::ADD;
  bbox_msg.id = 1;
  bbox_msg.type = visualization_msgs::Marker::LINE_LIST;
  bbox_msg.scale.x = 0.02;
  bbox_msg.scale.y = 0.02;
  bbox_msg.scale.z = 0.02;
  bbox_msg.color.r = 0.5;
  bbox_msg.color.g = 0;
  bbox_msg.color.b = 0.5;
  bbox_msg.color.a = 1.0;
  bbox_msg.pose.orientation.w = 1;
  bbox_msg.pose.orientation.x = 0;
  bbox_msg.pose.orientation.y = 0;
  bbox_msg.pose.orientation.z = 0;
  bbox_msg.pose.position.x = 0;
  bbox_msg.pose.position.y = 0;
  bbox_msg.pose.position.z = 0;

  bbox_msg2 = bbox_msg;
  bbox_msg2.id = 2;
  bbox_msg2.color.r = 0;
  bbox_msg2.color.g = 1;
  bbox_msg2.color.b = 0;
  bbox_msg2.ns = "center";

  geometry_msgs::Point pt0, pt1;
  for (int i = 0; i < hull_pcd.size(); i++) {
    auto p0 = hull_pcd.points[i];
    pt0.x = p0.x, pt0.y = p0.y, pt0.z = p0.z;
    auto p1 = hull_pcd.points[(i + 1) % hull_pcd.size()];
    pt1.x = p1.x, pt1.y = p1.y, pt1.z = p1.z;

    bool z = GetCrossDir(Eigen::Vector3f(p0.x, p0.y, p0.z), track_center,
                         Eigen::Vector3f(p1.x, p1.y, p1.z));
    if (!z) {  // purple
      bbox_msg.points.push_back(pt0);
      bbox_msg.points.push_back(pt1);
    } else {  // green
      bbox_msg2.points.push_back(pt0);
      bbox_msg2.points.push_back(pt1);
    }
  }
  bbox_msg.header.stamp = ros::Time::now();
  bbox_msg2.header.stamp = ros::Time::now();
  // ROS_INFO("<LocalPlanning::PublishConvexHullMarkers>: cross > 0 : %d / %d",
  //          (int)bbox_msg.points.size() / 2, (int)bbox_msg2.points.size() /
  //          2);
  if (bbox_msg.points.size() > 0) pub.publish(bbox_msg);
  if (bbox_msg2.points.size() > 0) pub.publish(bbox_msg2);
}

void LocalPlanning::PublishColorPointcloud(
    ros::Publisher &pub, pcl::PointCloud<pcl::PointXYZRGB> &pcd) {
  if (pcd.size() > 0) {
    std::lock_guard<std::mutex> lock(color_pcd_publisher_mutex_);
    sensor_msgs::PointCloud2 pcd_msg;
    pcl::toROSMsg(pcd, pcd_msg);
    pcd_msg.header.frame_id = map_frame_;
    pcd_msg.header.stamp = ros::Time::now();
    pub.publish(pcd_msg);
  }
}

void LocalPlanning::PublishVectorPoseMarker(ros::Publisher &pub,
                                            Eigen::Vector3f pos,
                                            Eigen::Vector3f dir) {
  Eigen::Quaternionf quat =
      Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0, 0, 1), dir);
  PublishVectorPoseMarker(pub, pos, quat);
}

void LocalPlanning::PublishVectorPoseMarker(ros::Publisher &pub,
                                            Eigen::Vector3f pos,
                                            Eigen::Quaternionf dir) {
  std::lock_guard<std::mutex> lock(vector_pose_publisher_mutex_);
  // publish marker of vector, rotate 90deg to point upward
  Eigen::Quaternionf quat =
      dir * Eigen::Quaternionf(0.7071068, 0, -0.7071068, 0);
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = map_frame_;
  msg.pose.position.x = pos.x();
  msg.pose.position.y = pos.y();
  msg.pose.position.z = pos.z();
  msg.pose.orientation.w = quat.w();
  msg.pose.orientation.x = quat.x();
  msg.pose.orientation.y = quat.y();
  msg.pose.orientation.z = quat.z();
  pub.publish(msg);
}

bool LocalPlanning::EstimateNormalVector(Eigen::Vector3f goal_position,
                                         float base_radius,
                                         NeighDomain &neigh_domain,
                                         SampleSet *track_samples,
                                         Eigen::Vector3f *goal_normal) {
  octomap::point3d goal_pos(goal_position.x(), goal_position.y(),
                            goal_position.z());
  octomap::OcTreeKey curr_key = local_octree_ptr_->coordToKey(goal_pos);

  // search for neighbour occupied voxels within the radius of vehicle
  int radius_scale = base_radius / local_octree_ptr_->getResolution() + 1;
  neigh_domain.decentralized_points.clear();

  auto start = local_octree_ptr_->begin_leafs_bbx(
      goal_pos - octomap::point3d(base_radius, base_radius, base_radius),
      goal_pos + octomap::point3d(base_radius, base_radius, base_radius));
  auto end = local_octree_ptr_->end_leafs_bbx();
  Eigen::Vector3f p1_p2, p3_p4;

  if (track_samples) {
    // p1_p2 = Eigen::Vector3f(track_samples->base_border_corners.at(1).x -
    //                             track_samples->base_border_corners.at(0).x,
    //                         track_samples->base_border_corners.at(1).y -
    //                             track_samples->base_border_corners.at(0).y,
    //                         0);
    // p3_p4 = Eigen::Vector3f(track_samples->base_border_corners.at(3).x -
    //                             track_samples->base_border_corners.at(2).x,
    //                         track_samples->base_border_corners.at(3).y -
    //                             track_samples->base_border_corners.at(2).y,
    //                         0);
  }
  int pt_cnt_ = 0;
  Eigen::Vector3f mu_(0, 0, 0);
  Eigen::Matrix3f sigma_ = Eigen::Matrix3f::Zero();

  // 记录maxZ, minZ
  // float maxZ = -1e9;
  // float minZ = 1e9;
  for (auto it = start; it != end; it++) {  // 耗时2ms
    octomap::OcTreeKey key = it.getKey();
    octomap::OcTreeNode *node = local_octree_ptr_->search(key);
    if (node && local_octree_ptr_->isNodeOccupied(node)) {
      auto p = local_octree_ptr_->keyToCoord(key);
      {
        float factor = 1 / static_cast<float>(pt_cnt_ + 1);
        Eigen::Vector3f diff_pt = Eigen::Vector3f(p.x(), p.y(), p.z()) - mu_;
        sigma_ = pt_cnt_ * factor * sigma_ +
                 pt_cnt_ * factor * factor * diff_pt * diff_pt.transpose();
        mu_ = mu_ + factor * diff_pt;
        ++pt_cnt_;
        // 进一步限制在车身包围框内
      }
      float x = p.x() - goal_pos.x();
      float y = p.y() - goal_pos.y();
      float z = p.z() - goal_pos.z();
      if (track_samples) {
        // Eigen::Vector3f p1_p(x - track_samples->base_border_corners.at(0).x,
        //                      y - track_samples->base_border_corners.at(0).y,
        //                      0);
        // Eigen::Vector3f p3_p(x - track_samples->base_border_corners.at(2).x,
        //                      y - track_samples->base_border_corners.at(2).y,
        //                      0);
        // if ((p1_p2.cross(p1_p).dot(p3_p4.cross(p3_p))) >= 0)
        neigh_domain.decentralized_points.push_back(pcl::PointXYZ(x, y, z));
      } else {
        neigh_domain.decentralized_points.push_back(pcl::PointXYZ(x, y, z));
      }
      // maxZ = std::max(p.z(), maxZ);
      // minZ = std::min(p.z(), minZ);
    }
  }

  // printf("<LocalPlanning::EstimateNormalVector>: neigh points: %d / %d\n",
  //        static_cast<int>(neigh_domain.decentralized_points.size()), pt_cnt_);

  if (neigh_domain.decentralized_points.size() == 0) {
    ROS_ERROR("<LocalPlanning::EstimateNormalVector>: no neighors found.");
    return false;
  }

  // float disZ = maxZ - minZ;
  // float dis_th = 0.84;
  // 2 * base_radius / fabs(maxZ - minZ) > sin(36 * M_PI / 180)
  // if (disZ > dis_th) {  // radius=0.6, angle=45°
  //   ROS_ERROR(
  //       "<LocalPlanning::EstimateNormalVector>: large height diff:
  //       %.2f/%.2f.", disZ, dis_th);
  //   return false;
  // }

  // estimate normal vector
  Eigen::Vector3f normal;
  if (goal_normal == NULL || goal_normal->norm() == 0) {
    Eigen::EigenSolver<Eigen::Matrix3f> es(sigma_);
    auto values = es.eigenvalues().real();
    auto vectors = es.eigenvectors().real();
    float min_eigen_val = 1e9;
    for (int i = 0; i < values.rows(); ++i) {
      if (values[i] < min_eigen_val) {
        min_eigen_val = values[i];
        normal = vectors.col(i);
      }
    }
  } else {
    normal = *goal_normal;
  }
  // correct direction
  if (normal.z() < 0) normal *= -1.0;
  Eigen::Quaternionf rotation =
      Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0, 0, 1), normal);

  Eigen::Vector3f euler;
  if (!CheckEulerConstraint(Eigen::Quaternionf::Identity(), rotation, euler, 45,
                            45)) {
    ROS_ERROR(
        "<LocalPlanning::EstimateNormalVector>: surface slope is out of bound. "
        "Normal[%.3f,%.3f,%.3f]",
        normal.x(), normal.y(), normal.z());
    return false;
  }

  // fill the class
  neigh_domain.base_center = goal_position;
  neigh_domain.base_radius = base_radius;
  neigh_domain.normal_vector = normal;
  neigh_domain.normal_rotation = rotation;

  // publish marker of normal vector
  PublishVectorPoseMarker(goal_norm_publisher_, goal_position, rotation);

  return true;
}

Eigen::Vector3f LocalPlanning::toEulerAngle(const Eigen::Quaternionf &q) {
  double roll, pitch, yaw;
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw = atan2(siny_cosp, cosy_cosp);
  return Eigen::Vector3f(roll, pitch, yaw);
}

bool LocalPlanning::CheckEulerConstraint(Eigen::Quaternionf pose,
                                         Eigen::Quaternionf base_dir,
                                         Eigen::Vector3f &euler, float roll_th,
                                         float pitch_th) {
  return true;
  Eigen::Quaternionf q = pose * base_dir;
  euler = toEulerAngle(q);
  euler *= 180.0 / M_PI;

  Eigen::Vector3f x(1, 0, 0);
  Eigen::Vector3f x_t =
      pose.toRotationMatrix() * (base_dir.toRotationMatrix() * x);
  Eigen::Vector3f dis = x_t;

  float real_pitch =
      atan2(dis.z(), sqrt(dis.x() * dis.x() + dis.y() * dis.y())) * 180.0 /
      M_PI;
  printf(
      "<CheckEulerConstraint>: R=%.2f/%.2f, "
      "P=%.2f/%.2f, Y=%.2f, ~Dir=[%.2f,%.2f,%.2f], Pitch2D=%.2f\n",
      euler.x(), roll_th, euler.y(), pitch_th, euler.z(), x_t.x(), x_t.y(),
      x_t.z(), real_pitch);
  if (fabs(euler.x()) > roll_th || fabs(euler.y()) > pitch_th) {
    ROS_ERROR("<CheckEulerConstraint>: Slope is out of bound.");
    return false;
  }
  euler.y() = real_pitch;

  return true;
}

bool LocalPlanning::GetClosestPoint(octomap::point3d curr_pos,
                                    float search_radius,
                                    octomap::point3d &closest_pos) {
  octomap::OcTreeKey curr_key = local_octree_ptr_->coordToKey(curr_pos);
  int radius_scale = search_radius / local_octree_ptr_->getResolution() + 1;
  float min_dis = 1e9;
  bool isFound = false;

  search_radius += local_octree_ptr_->getResolution();
  auto start = local_octree_ptr_->begin_leafs_bbx(
      curr_pos - octomap::point3d(search_radius, search_radius, search_radius),
      curr_pos + octomap::point3d(search_radius, search_radius, search_radius));
  auto end = local_octree_ptr_->end_leafs_bbx();
  for (auto it = start; it != end; it++) {  // 8ms
    octomap::OcTreeKey key = it.getKey();
    octomap::OcTreeNode *node = local_octree_ptr_->search(key);
    if (node && local_octree_ptr_->isNodeOccupied(node)) {
      auto p = local_octree_ptr_->keyToCoord(key);
      float dis = (p - curr_pos).norm();
      if (dis < min_dis) {
        min_dis = dis;
        isFound = true;
        closest_pos = p;
      }
    }
  }

  // for (int dx = -radius_scale; dx <= radius_scale; dx++) {  // 30ms
  //   for (int dy = -radius_scale; dy <= radius_scale; dy++) {
  //     for (int dz = -radius_scale; dz <= radius_scale; dz++) {
  //       octomap::OcTreeKey key;
  //       key.k[0] = curr_key[0] + dx;
  //       key.k[1] = curr_key[1] + dy;
  //       key.k[2] = curr_key[2] + dz;
  //       octomap::OcTreeNode *node = local_octree_ptr_->search(key);
  //       if (node && local_octree_ptr_->isNodeOccupied(node)) {
  //         auto p = local_octree_ptr_->keyToCoord(key);
  //         float dis = (p - curr_pos).norm();
  //         if (dis < min_dis) {
  //           min_dis = dis;
  //           isFound = true;
  //           closest_pos = p;
  //         }
  //       }
  //     }
  //   }
  // }

  return isFound;
}
