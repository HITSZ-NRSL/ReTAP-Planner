#pragma once
#include <math.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/surface/convex_hull.h>

#include <algorithm>
#include <functional>
#include <vector>

#include "libqhullcpp/Qhull.h"
#include "libqhullcpp/QhullFacetList.h"

template <typename Point>
class QuickHull2D {
  using vPoint = std::vector<Point>;

 public:
  QuickHull2D(vPoint vec) {  // NOLINT
    input = vec;

    // Sort with x/y
    std::sort(input.begin(), input.end(), [&](Point &p1, Point &p2) -> bool {
      return p1.x > p2.x || (p1.x == p2.x && p1.y > p2.y);
    });
  }

  /* // Deprecated.
  // 对s[]中的n个点，提取凸包点存入ch[]，返回凸包点数v
  // https://zhuanlan.zhihu.com/p/264334490
  vPoint getConvexHull() {
    auto s = input;
    auto ch = input;
    int n = input.size();
    int v = 0;
    for (int i = 0; i < n; i++) {
      while (v > 1 &&
             sgn(cross2(ch[v - 1] - ch[v - 2], s[i] - ch[v - 2])) <= 0) {
        v -= 1;
      }
      ch[v++] = s[i];
    }
    int j = v;
    for (int i = n - 2; i >= 0; i--) {
      while (v > j &&
             sgn(cross2(ch[v - 1] - ch[v - 2], s[i] - ch[v - 2])) <= 0) {
        v -= 1;
      }
      ch[v++] = s[i];
    }
    if (n > 1) v -= 1;

    ch.erase(ch.begin() + v, ch.end());
    hull_quick = ch;
    return hull_quick;
  }
  */

  vPoint getConvexHull_Qhull() {
    hull_quick.clear();
    std::vector<double> allCoords;
    for (int i = 0; i < input.size(); i++) {
      allCoords.push_back(input[i].x);
      allCoords.push_back(input[i].y);
    }
    double *buffer = new double[allCoords.size()];
    memcpy(buffer, &allCoords[0], allCoords.size() * sizeof(double));

    char inputComment2[] = "";
    char qhullCommand2[] = "";

    mean_x = 0, mean_y = 0;
    try {
      orgQhull::Qhull qhull;
      qhull.runQhull(inputComment2, 2, input.size(), buffer, qhullCommand2);
      auto vertices = qhull.vertexList();
      if (vertices.size() == 0) return hull_quick;
      for (auto p : vertices) {
        double *coord = p.point().coordinates();
        Point pt;
        pt.x = coord[0], pt.y = coord[1], pt.z = 0;
        hull_quick.push_back(pt);
        mean_x += coord[0];
        mean_y += coord[1];
      }
      mean_x /= vertices.size();
      mean_y /= vertices.size();
    } catch (orgQhull::QhullError &e) {
      ROS_ERROR("<QuickHull2D::getConvexHull_Qhull>: execption: \n%s", e.what());
      for (auto p : input) {
        ROS_ERROR(
            "<QuickHull2D::getConvexHull_Qhull>: point [%.3f, %.3f, %.3f]\n",
            p.x, p.y, p.z);
      }
    }
    delete[] buffer;
    // printf("<QuickHull2D::getConvexHull_Qhull>: sort %d vertices.\n",
    //        (int)hull_quick.size());

    // 减掉均值，排序围绕(0,0)旋转，避免tan算错
    for (auto &p : hull_quick) {
      p.x -= mean_x;
      p.y -= mean_y;
    }
    std::sort(hull_quick.begin(), hull_quick.end(), azimuthCmp);
    for (auto &p : hull_quick) {
      p.x += mean_x;
      p.y += mean_y;
      // ROS_INFO("<getConvexHull_Qhull>: vertex [%.3f, %.3f]", p.x, p.y);
    }

    return hull_quick;
  }

  /* // Deprecated. 维度设为3，提示缺少1个维度；设为2，段错误
  vPoint getConvexHull_PCL() {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < input.size(); i++) {
      auto p = input[i];
      cloud.push_back(pcl::PointXYZ(p.x, p.y, p.z()));
    }
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud.makeShared());
    hull.setComputeAreaVolume(true);
    hull.setDimension(3);
    std::vector<pcl::Vertices> polygons;
    pcl::PointCloud<pcl::PointXYZ> surface_hull;
    hull.reconstruct(surface_hull, polygons);

    hull_quick.clear();
    for (int i = 0; i < surface_hull.size(); i++) {
      auto p = surface_hull[i];
      hull_quick.push_back(Point(p.x, p.y, p.z));
    }
  }
*/

  /* // Deprecated. 输入点数多时，会出现大量重复点
  // https://blog.csdn.net/shungry/article/details/104342366
  vPoint getQuickHull() {
    hull_quick.clear();
    int min_x = 0, max_x = 0;
    for (int i = 0; i < input.size(); i++) {
      if (input[i].x < input[min_x].x) {
        min_x = i;
      }
      if (input[i].x > input[max_x].x) {
        max_x = i;
      }
    }

    hull_quick.push_back(input[min_x]);
    hull_quick.push_back(input[max_x]);

    vPoint left, right;
    for (int i = 0; i < input.size(); i++) {
      if (i == min_x || i == max_x) {
        continue;
      }
      // int temp = get_sign(input[min_x], input[max_x], input[i]);
      int temp = sgn(cross3(input[min_x], input[max_x], input[i]));
      if (temp == 0) {
        continue;
      }
      if (temp < 0) {
        left.push_back(input[i]);
      } else {
        right.push_back(input[i]);
      }
    }
    qhullrecur(right, input[min_x], input[max_x]);
    qhullrecur(left, input[max_x], input[min_x]);

    // auto last_ptr = std::unique(hull_quick.begin(), hull_quick.end());
    // hull_quick.erase(last_ptr, hull_quick.end());
    // std::sort(hull_quick.begin(), hull_quick.end(), azimuthCmp);
    for (int i = 0; i < hull_quick.size(); i++) {
      auto p = hull_quick[i];
      ROS_INFO("<QuickHull2D::getHullVertices>: [%d] (%.3f, %.3f, %.3f)", i,
               p.x, p.y, p.z());
    }
    return hull_quick;
  }
  */

  double getHullArea() {
    double s = 0;
    for (int i = 0; i < hull_quick.size() - 1; i++) {
      s += fabs(cross2(hull_quick[i], hull_quick[i + 1]));  // 叉积
    }
    return 0.5 * s;
  }

  // Point getHullCenter() {
  //   Point center;
  //   center.x = 0;
  //   center.y = 0;
  //   for (auto p : input) {
  //     center.x += p.x;
  //     center.y += p.y;
  //   }
  //   center.x /= input.size();
  //   center.y /= input.size();
  //   return center;
  // }

  // O(log(n))
  bool inConvexPoly(Point p) {
    int n = hull_quick.size();
    for (int i = 0; i < n; i++) {
      Point pt1;
      pt1.x = hull_quick[i].x - p.x;
      pt1.y = hull_quick[i].y - p.y;
      Point pt2;
      pt2.x = hull_quick[(i + 1) % n].x - p.x;
      pt2.y = hull_quick[(i + 1) % n].y - p.y;

      if (sgn(cross2(pt1, pt2)) < 0) return false;
      // else if (OnSeg(a, Line(p[i], p[(i + 1) % n])))
      //   return 0;
    }
    return true;
  }

 private:
  vPoint hull_quick;
  vPoint input;
  float mean_x, mean_y;

  static bool azimuthCmp(Point a, Point b) {
    if (atan2(a.y, a.x) != atan2(b.y, b.x))
      return atan2(a.y, a.x) < atan2(b.y, b.x);
    else
      return a.x < b.x;
  }

  // c到ab线段的距离
  double cross3(Point a, Point b, Point c) {
    return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
  }

  double cross2(Point a, Point b) { return a.x * b.y - a.y * b.x; }

  int sgn(double a) {
    if (fabs(a) < 1e-3) return 0;
    if (a > 0) return 1;
    return -1;
  }

  double dist(Point a, Point b, Point c) { return fabs(cross3(a, b, c)); }

  /* // Deprecated with getQuickHull
  void qhullrecur(const vPoint &temp, Point a, Point b) {
    if (temp.size() == 0) {
      return;
    }

    int max_ind = -1;
    int max_dis = 0;
    for (int i = 0; i < temp.size(); i++) {
      if (sgn(cross3(a, b, temp[i])) > 0) {  // get_right
        if (max_dis < dist(a, b, temp[i])) {
          max_ind = i;
          max_dis = dist(a, b, temp[i]);
        }
      }
    }

    if (max_ind == -1) return;

    Point c = temp[max_ind];
    hull_quick.push_back(c);

    vPoint acr, bcr;
    for (int i = 0; i < temp.size(); i++) {
      if (sgn(cross3(a, c, temp[i])) > 0) {  // get_right
        acr.push_back(temp[i]);
      }
      if (sgn(cross3(c, b, temp[i])) > 0) {  // get_right
        bcr.push_back(temp[i]);
      }
    }
    qhullrecur(acr, a, c);
    qhullrecur(bcr, c, b);
  }
  */
};

template <typename Point>
class MinBoundingRectangle {
  using vPoint = std::vector<Point>;
  typedef struct BoundingBox {
    std::vector<Point> rectangle;  // left_down, right_down, right_up, left_up
    double area;
    double orientation;
  } BoundingBox;

 public:
  MinBoundingRectangle(vPoint vec) {  // NOLINT
    input = vec;
  }

  std::vector<float> getEdgesAngles() {
    std::vector<float> angles;
    for (size_t i = 0; i < input.size(); i++) {
      float angle = atan2(input[i].y - input[(i + 1) % input.size()].y,
                          input[i].x - input[(i + 1) % input.size()].x);

      // set all angles to [0, 90)
      angle = (angle >= 0.0) ? angle : (angle += (2. * M_PI));
      angle = fmod(angle, (float)(M_PI / 2.0));  // NOLINT
      angles.push_back(angle);
    }
    return angles;
  }

  Point rotatingPoint(Point p, float theta) {
    Point q;
    q.x = cos(theta) * p.x - sin(theta) * p.y;
    q.y = sin(theta) * p.x + cos(theta) * p.y;
    return q;
  }

  std::vector<Point> rotatingPoints(std::vector<Point> points, float theta) {
    std::vector<Point> points_r;
    for (size_t i = 0; i < points.size(); i++) {
      points_r.push_back(rotatingPoint(points[i], theta));
    }
    return points_r;
  }

  BoundingBox getMinAreaRectange(vPoint points) {
    BoundingBox bbox;
    if (points.size() < 1) {
      std::cout << "getMinAreaRectange----points < 1" << endl;
      return bbox;
    }

    double min_x = points[0].x;
    double max_x = points[0].x;
    double min_y = points[0].y;
    double max_y = points[0].y;
    for (size_t i = 1; i < points.size(); i++) {
      if (points[i].x < min_x) min_x = points[i].x;
      if (points[i].x > max_x) max_x = points[i].x;
      if (points[i].y < min_y) min_y = points[i].y;
      if (points[i].y > max_y) max_y = points[i].y;
    }

    Point p;
    p.x = min_x, p.y = min_y;
    bbox.rectangle.push_back(p);
    p.x = max_x, p.y = min_y;
    bbox.rectangle.push_back(p);
    p.x = max_x, p.y = max_y;
    bbox.rectangle.push_back(p);
    p.x = min_x, p.y = max_y;
    bbox.rectangle.push_back(p);

    bbox.area = (max_x - min_x) * (max_y - min_y);

    return bbox;
  }

  // rotating calipers
  float getMinimumBoundingRectangle() {
    std::vector<float> angles = getEdgesAngles();
    std::sort(angles.begin(), angles.end());
    angles.erase(unique(angles.begin(), angles.end()), angles.end());

    std::vector<BoundingBox> bboxes;
    float area_min = 1e9;
    int index = -1;
    for (size_t i = 0; i < angles.size(); i++) {
      double theta = angles[i];
      std::vector<Point> points_r =
          rotatingPoints(input, -1.0 * theta);  // anti-clock wise
      BoundingBox bbox = getMinAreaRectange(points_r);
      bbox.orientation = theta;
      bboxes.push_back(bbox);

      if (bbox.area < area_min) {
        area_min = bbox.area;
        index = i;
      }
    }

    float dis_x1 = bboxes[index].rectangle[1].x - bboxes[index].rectangle[0].x;
    float dis_y1 = bboxes[index].rectangle[1].y - bboxes[index].rectangle[0].y;
    float dis_x2 = bboxes[index].rectangle[2].x - bboxes[index].rectangle[1].x;
    float dis_y2 = bboxes[index].rectangle[2].y - bboxes[index].rectangle[1].y;
    float length = sqrt(dis_x1 * dis_x1 + dis_y1 * dis_y1);
    float width = sqrt(dis_x2 * dis_x2 + dis_y2 * dis_y2);
    // std::cout << "<QuickHull2D::getMinimumBoundingRectangle>: area_min = " <<
    // area_min
    //           << ", rect len = " << length << ", width = " << width
    //           << std::endl;
    return area_min;
  }

  vPoint input;
};
