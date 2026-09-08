#ifndef ASTAR_H
#define ASTAR_H
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/Path.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <limits>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "astar_3d/gl_const.h"
#include "astar_3d/list.h"
#include "astar_3d/node.h"
#include "mapping_module/world_representation/world_representation.h"

// #include "astar_3d/astar_base.h"

struct SearchResult {
  bool pathfound;
  float pathlength;
  const NodeList *lppath;
  const NodeList *hppath;
  unsigned int nodescreated;
  unsigned int numberofsteps;
  double time;
  SearchResult() {
    pathfound = false;
    pathlength = 0;
    lppath = NULL;
    hppath = NULL;
    nodescreated = 0;
    numberofsteps = 0;
    time = 0;
  }
};

class EnvironmentOptions {
 public:
  EnvironmentOptions() {}

  void set(int MT, bool AS, double LC, double DC, int AD, bool AC) {
    metrictype = MT;
    allowsqueeze = AS;
    linecost = LC;
    diagonalcost = DC;
    allowdiagonal = AD;
    allowcutcorners = AC;
  }

  int metrictype;
  bool allowsqueeze;
  bool allowcutcorners;
  double linecost;
  double diagonalcost;
  int allowdiagonal;
  bool useresetparent;
};

class Map {
 public:
  Map() {}
  ~Map() {}

  octomap::point3d IndexToCoord(int i, int j, int h) {
    float x = ((i - offset_i) + 0.5) * resolution;
    float y = ((j - offset_j) + 0.5) * resolution;
    float z = ((h - offset_h) + 0.5) * resolution;

    // std::cout << "<Astar::IndexToCoord>: Index["
    //           << Eigen::Vector3f(i, j, h).transpose() << "] -> Coord["
    //           << Eigen::Vector3f(x, y, z).transpose() << "]" << std::endl;
    return octomap::point3d(x, y, z);
  }

  void CoordToIndex(Eigen::Vector3f p, int &i, int &j, int &h) {  // NOLINT
    p /= resolution;
    i = p.x() + offset_i;
    j = p.y() + offset_j;
    h = p.z() + offset_h;
  }

  void CoordToIndex(octomap::point3d p, int &i, int &j, int &h) {  // NOLINT
    p /= resolution;
    i = p.x() + offset_i;
    j = p.y() + offset_j;
    h = p.z() + offset_h;
  }

  // bool CellIsObstacle(int i, int j, int h) const;
  // bool CellOnGrid (int i, int j, int height) const;
  // int  getValue(int i, int j) const;

  void setStartPoint(Eigen::Vector3f p) {
    CoordToIndex(p, start_i, start_j, start_h);
  }

  void setGoalPoint(Eigen::Vector3f p) {
    CoordToIndex(p, goal_i, goal_j, goal_h);
  }

  void setMapSize(int h, int w, int a) {
    // i->height, j->width, z->altitude
    height = h;
    width = w;
    altitude = a;
    std::cout << "<Astar::setMapSize>: " << Eigen::Vector3f(h, w, a).transpose()
              << std::endl;
  }

  void setCenterOffset(int x, int y, int z) {
    offset_i = x;
    offset_j = y;
    offset_h = z;
    std::cout << "<Astar::setCenterOffset>: "
              << Eigen::Vector3f(x, y, z).transpose() << std::endl;
  }

  void setResolution(float res) {
    resolution = res;
    std::cout << "<Astar::setResolution>: " << resolution << std::endl;
  }

 public:
  int height, width, altitude;
  int min_altitude_limit, max_altitude_limit;  // The lowest and highest
                                               // possible altitude for the path
  int start_i, start_j, start_h;
  int goal_i, goal_j, goal_h;
  int offset_i, offset_j, offset_h;
  float resolution;
};

class Astar {
 public:
  using ElevationMap = std::unordered_map<octomap::OcTreeKey, octomap::key_type,
                                          octomap::OcTreeKey::KeyHash>;

  Astar(octomap::IgTree *tree, float h_w, float t_w, float g_w) {
    octree_ptr_ = tree;

    float res = octree_ptr_->getResolution();
    octomap::point3d bbx_max(300, 300, 100);     // = octree_ptr_->getBBXMax();
    octomap::point3d bbx_min(-300, -300, -100);  // = octree_ptr_->getBBXMin();
    map.setCenterOffset(-floor(bbx_min.x() / res), -floor(bbx_min.y() / res),
                        -floor(bbx_min.z() / res));
    map.setMapSize(floor(bbx_max.x() / res) - floor(bbx_min.x() / res),
                   floor(bbx_max.y() / res) - floor(bbx_min.y() / res),
                   floor(bbx_max.z() / res) - floor(bbx_min.z() / res));
    map.setResolution(res);

    // Eigen::Vector3f start_point(-1.728, 1.231, -0.797);
    // Eigen::Vector3f goal_point(15, 10.4, 2.6);  // (6.083, 6.433, -0.797);
    // SetStartAndGoal(start_point, goal_point);

    hweight = h_w;  // 0.3
    tweight = t_w;  // 10
    gweight = g_w;  // 1
    breakingties = CN_SP_BT_GMAX;

    printf(
        "<Astar::Astar>: traverse_weight: %.3f, hueristic_weight: %.3f, "
        "distance_weight: %.3f\n",
        tweight, hweight, gweight);
    options.set(CN_SP_MT_CHEB, CN_SP_AS_FALSE, CN_MC_LINE, CN_MC_DIAG,
                CN_SP_AD_TRUE, CN_SP_AC_FALSE);

    elevation_matrix = new (int[map.height * map.width]);
    for (int i = 0; i < map.height * map.width; i++) elevation_matrix[i] = -1e3;
  }

  void SetStartAndGoal(Eigen::Vector3f start_point,
                       Eigen::Vector3f goal_point) {
    map.setGoalPoint(goal_point);
    map.setStartPoint(start_point);
    std::cout
        << "<Astar::SetStartAndGoal>: \n\tstart: " << start_point.transpose()
        << ", key: "
        << Eigen::Vector3f(map.start_i, map.start_j, map.start_h).transpose()
        << std::endl
        << "\tgoal: " << goal_point.transpose() << ", key: "
        << Eigen::Vector3f(map.goal_i, map.goal_j, map.goal_h).transpose()
        << std::endl;
  }

  ~Astar() { delete[] elevation_matrix; }

  virtual float CellTraversable(octomap::OcTreeKey key) {
    octomap::IgTreeNode *node = octree_ptr_->search(key);
    if (!node || !octree_ptr_->isNodeOccupied(node) || node->getCollision()) {
      return 1e9;
    }
    float value = node->getTraversability();

    // std::cout << "<Astar::CellIsTraversable>: " <<
    // Eigen::Vector3f(i,j,h).transpose() << " : " << value << std::endl;
    return value;
  }

  virtual float CellTraversable(int i, int j, int h) {
    octomap::OcTreeKey key = octree_ptr_->coordToKey(map.IndexToCoord(i, j, h));
    return CellTraversable(key);
  }

  nav_msgs::Path startSearch() {
    std::cout << "<Astar::startSearch>: ..." << std::endl;
    ros::WallTime t1 = ros::WallTime::now();
    // 初始化一个行列表，长度为行数，存储该行处理过的
    open = new std::unordered_map<uint_least32_t, node_t>[map.height + 1];
    // 初始化一个行向量，存储每行的最小值的key
    openMinimums = std::vector<int_least64_t>(map.height + 1, -1);

    node_t curNode;
    curNode.i = map.start_i;
    curNode.j = map.start_j;
    curNode.z = map.start_h;
    curNode.g = 0;
    // 计算从current到goal的距离
    curNode.H =
        computeHFromCellToCell(curNode.i, curNode.j, curNode.z, map.goal_i,
                               map.goal_j, map.goal_h, options);
    curNode.F = hweight * curNode.H;
    curNode.T = 0;
    curNode.parent = nullptr;

    // 判断newNode是否在当前行内open[idx]，没有则插入，记录open格子的数量
    // 若newNode的F值，比当前行最小值openMinimums[idx]，则更新该值
    // 处理过的，则为open
    addOpen(curNode, curNode.get_id(map.height, map.width));
    bool pathfound = false;
    const node_t *curIt;
    size_t closeSize = 0;
    openSize = 1;
    while (!stopCriterion() && ros::ok()) {  // openSize != 0
      // 遍历每一行，从openMinimums[i]获得最小key，
      // 并从open[i][key]取出F值，在每一行的最小值中找到最小的
      curNode = findMin(map.height);

      uint_least32_t id = curNode.i * map.width + curNode.j +
                          map.height * map.width * curNode.z;
      close.insert({id, curNode});  // 所有open中最小的，则为close
      ++closeSize;

      // 从open集合中，删掉curNode（最小的），更新openMinimums[]；避免下次重复选中
      deleteMin(curNode, curNode.get_id(map.height, map.width));
      --openSize;
      if (curNode.i == map.goal_i && curNode.j == map.goal_j &&
          fabs(curNode.z - map.goal_h) <= 2) {  //
        pathfound = true;
        break;
      }

      // 在curNode的26邻域内，在map中找到traversable node
      // 且不在close集合中（避免形成环），计算其距离g，将其存入output[]并返回
      std::list<node_t> successors = findSuccessors(curNode, map, options);
      id = curNode.i * map.width + curNode.j +
           map.height * map.width * curNode.z;
      auto parent = &(close.find(id)->second);

      auto it = successors.begin();
      while (it != successors.end()) {
        // 遍历邻域26 可通行的格子，计算H、F值，更新open集合
        it->parent = parent;
        it->H = computeHFromCellToCell(it->i, it->j, it->z, map.goal_i,
                                       map.goal_j, map.goal_h, options);
        *it = resetParent(*it, *it->parent, map, options);
        it->F = it->g + hweight * it->H + tweight * it->T * 1 / map.resolution;
        addOpen(*it, it->get_id(map.height, map.width));
        it++;
      }
    }

    nav_msgs::Path path;
    sresult.pathfound = false;
    sresult.nodescreated = closeSize + openSize;
    sresult.numberofsteps = closeSize;
    if (!stopCriterion() && pathfound) {  // 真正到达目标点
      sresult.pathfound = true;
      makePrimaryPath(curNode);  // 对终止节点，递归找爹得到路径lppath
      std::cout << "<Astar::startSearch>: path found with "
                << sresult.hppath->List.size() << " waypoints" << std::endl;
    } else {
      if (stopCriterion()) {  // open list is empty
        ROS_ERROR("<Astar::startSearch>: stop criterion, no path found.");
        // makeSecondaryPath();  // find the nearest node to the goal, and
        // generate path
      } else {  // timeout without feasible path
                // replan ...
        ROS_ERROR("<Astar::startSearch>: timeout, no path found.");
      }
      delete[] open;
      return path;  // 必须返回，否则后面读取sresult.hppath->List报错
    }

    for (auto &it : sresult.hppath->List) {
      octomap::point3d p = map.IndexToCoord(it.i, it.j, it.z);
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      Eigen::Vector3f normal(0, 0, 1);
      // normal = octree_ptr_->search(octree_ptr_->coordToKey(p))
      //                   ->getFuseEigen()
      //                   .min_eigen_vec;
      // normal.normalize();

      // if (normal.z() < 0) normal *= -1.0;
      // pose.pose.orientation.x = normal.x();
      // pose.pose.orientation.y = normal.y();
      // pose.pose.orientation.z = normal.z();
      path.poses.push_back(pose);
      std::cout << "<Astar::startSearch>: waypoint coord: "
                << Eigen::Vector3f(p.x(), p.y(), p.z()).transpose()
                << ", key: " << Eigen::Vector3f(it.i, it.j, it.z).transpose()
                << ", normal: " << normal.transpose() << std::endl;
    }

    ros::WallDuration d = ros::WallTime::now() - t1;
    sresult.time = d.toSec();
    std::cout << "<Astar::startSearch>: timecost: " << sresult.time
              << " s. Found [" << pathfound
              << "]. Node num: " << sresult.hppath->List.size() << std::endl;

    delete[] open;
    return path;
  }

  // 判断newNode是否在当前行内open[idx]，没有则插入，记录open格子的数量；
  // 判断当前行最小值openMinimums[idx]，若newNode的F值更小，则更新该值
  void addOpen(node_t newNode, uint_least32_t key) {
    bool inserted = false;
    size_t idx = newNode.i;
    if (open[idx].find(key) != open[idx].end()) {  // 已经处理过，代价更小则更新
      if (newNode.F < open[idx][key].F) {
        open[idx][key] = newNode;
        inserted = true;
      }
    } else {
      open[idx][key] = newNode;
      inserted = true;
      ++openSize;
    }

    if (open[idx].size() == 1) {
      openMinimums[idx] = key;
    } else {
      node_t min_node = open[idx][openMinimums[idx]];
      if (inserted && newNode.F <= min_node.F) {
        if (newNode.F == min_node.F) {
          switch (breakingties) {
            default:
            case CN_SP_BT_GMAX: {
              if (newNode.g >= min_node.g) {
                openMinimums[idx] = key;
              }
              break;
            }
            case CN_SP_BT_GMIN: {
              if (newNode.g <= min_node.g) {
                openMinimums[idx] = key;
              }
              break;
            }
          }
        } else {
          openMinimums[idx] = key;
        }
      }
    }
  }

  bool stopCriterion() {
    if (openSize == 0) {
      std::cout << "OPEN list is empty!" << std::endl;
      return true;
    }
    return false;
  }

  // 从遍历map.height，openMinimums[i]中取出最小key，并从open[i][key]取出F值，找到最小的
  node_t findMin(int size) {
    node_t min, cur_node;
    min.F = std::numeric_limits<double>::infinity();
    for (int i = 0; i < size; i++) {
      if (!open[i].empty()) {
        cur_node = open[i][openMinimums[i]];
        if (cur_node.F <= min.F) {
          if (cur_node.F == min.F) {
            switch (breakingties) {
              default:
              case CN_SP_BT_GMAX: {
                if (cur_node.g >= min.g) {
                  min = cur_node;
                }
                break;
              }
              case CN_SP_BT_GMIN: {
                if (cur_node.g <= min.g) {
                  min = cur_node;
                }
                break;
              }
            }
          } else {
            min = cur_node;
          }
        }
      }
    }
    return min;
  }

  // 从open[minNode.i]集合中，删掉key，更新openMinimums[]
  void deleteMin(node_t minNode, uint_least32_t key) {
    size_t idx = minNode.i;
    open[idx].erase(key);
    node_t min_node;
    min_node.F = std::numeric_limits<float>::infinity();
    if (!open[idx].empty()) {
      for (auto it = open[idx].begin(); it != open[idx].end(); ++it) {
        if (it->second.F <= min_node.F) {
          if (it->second.F == min_node.F) {
            switch (breakingties) {
              default:
              case CN_SP_BT_GMAX: {
                if (it->second.g >= min_node.g) {
                  openMinimums[idx] = it->first;
                  min_node = it->second;
                }
                break;
              }
              case CN_SP_BT_GMIN: {
                if (it->second.g <= min_node.g) {
                  openMinimums[idx] = it->first;
                  min_node = it->second;
                }
                break;
              }
            }
          } else {
            openMinimums[idx] = it->first;
            min_node = it->second;
          }
        }
      }
    }
  }

  // 在curNode的26邻域内，在map中找到traversable node
  // 且不在close集合中，计算其g，将其存入output[]并返回
  virtual std::list<node_t> findSuccessors(node_t curNode, Map &map,
                                           EnvironmentOptions &options) {
    node_t newNode;
    std::list<node_t> output;
    for (int i = -1; i <= 1; ++i) {
      for (int j = -1; j <= 1; ++j) {
        for (int h = -1; h <= 1; ++h) {
          if (i == 0 && j == 0 && h == 0) continue;

          // && map.CellOnGrid(curNode.i + i, curNode.j + j, curNode.z + h)
          float traversability =
              CellTraversable(curNode.i + i, curNode.j + j, curNode.z + h);
          if (traversability >= 1) continue;

          if (options.allowdiagonal == CN_SP_AD_FALSE &&
              abs(i) + abs(j) + abs(h) > 1) {
            continue;
          }
          // if (options.allowsqueeze == CN_SP_AS_FALSE && (i != 0 && j != 0)) {
          //   int min_obs_height =
          //       std::min(map.getValue(curNode.i, curNode.j + j),
          //                map.getValue(curNode.i + i, curNode.j));
          //   if (min_obs_height > std::max(curNode.z, curNode.z + h)) {
          //     continue;
          //   }
          // }
          // if (options.allowcutcorners == CN_SP_AC_FALSE &&
          //     (map.CellIsObstacle(curNode.i + i, curNode.j + j, curNode.z) ||
          //      map.CellIsObstacle(curNode.i + i, curNode.j, curNode.z + h) ||
          //      map.CellIsObstacle(curNode.i, curNode.j + j, curNode.z + h)))
          //      {
          //   continue;
          // }
          newNode.i = curNode.i + i;
          newNode.j = curNode.j + j;
          newNode.z = curNode.z + h;
          uint_least32_t id = newNode.i * map.width + newNode.j +
                              map.height * map.width * newNode.z;
          if (close.find(id) == close.end()) {
            newNode.g = curNode.g + MoveCost(curNode.i, curNode.j, curNode.z,
                                             curNode.i + i, curNode.j + j,
                                             curNode.z + h, options);
            newNode.T = curNode.T + traversability;
            output.push_back(newNode);
          }
        }
      }
    }
    return output;
  }

  node_t resetParent(node_t current, node_t parent, const Map &map,
                     const EnvironmentOptions &options) {
    return current;
  }

  double MoveCost(int start_i, int start_j, int start_h, int fin_i, int fin_j,
                  int fin_h, const EnvironmentOptions &options) {
    // Assuming that we work in a Euclidean space
    int diff =
        abs(start_i - fin_i) + abs(start_j - fin_j) + abs(start_h - fin_h);
    switch (diff) {
      case 1:
        return options.linecost;
      case 2:
        return M_SQRT2 * options.linecost;
      case 3:
        return sqrt(3) * options.linecost;
      default:
      case 0:
        return 0;
    }
  }

  // 递归找爹，形成路径存入lppath和sresult.hppath
  void makePrimaryPath(node_t curNode) {
    node_t current = curNode;
    lppath.List.clear();
    while (current.parent != nullptr) {
      lppath.List.push_front(current);
      current = *current.parent;
    }
    lppath.List.push_front(current);
    // lppath顺序为起点->终点
    sresult.hppath = &lppath;
    sresult.pathlength = curNode.g;
  }

  // 对目标的最近邻节点，递归找爹，形成路径存入lppath和sresult.hppath
  void makeSecondaryPath() {
    node_t end_node;
    float max_cost = -1e9;
    std::unordered_map<uint_least32_t, node_t>::const_iterator iter;
    for (iter = close.begin(); iter != close.end(); iter++) {
      if (iter->second.F > max_cost) {
        max_cost = iter->second.F;
        end_node = iter->second;
      }
    }

    if (max_cost > -1e9) {
      std::cout << "close.size() = " << close.size() << std::endl;
      makePrimaryPath(end_node);
      // sresult.pathlength += (nearest_dis * map.resolution);  //
      // 增加近路的长度
    }
  }

  // 精简路径，只保留移动方向变化的节点，存入hppath
  void makeTopologyPath(node_t curNode) {
    std::list<node_t>::const_iterator iter = lppath.List.begin();
    node_t cur, next, move;
    hppath.List.push_back(*iter);
    while (iter != --lppath.List.end()) {  // 遍历Primary Path
      cur = *iter;
      iter++;
      next = *iter;
      move.i = next.i - cur.i;
      move.j = next.j - cur.j;
      move.z = next.z - cur.z;
      iter++;  // the next after the next
      if ((iter->i - next.i) != move.i || (iter->j - next.j) != move.j ||
          (iter->z - next.z) != move.z)
        hppath.List.push_back(*(--iter));  // 只保留移动方向变化的节点
      else
        iter--;
    }
    sresult.hppath = &hppath;
  }

  // 计算从start到fin的距离，Manh、Cheb、Eucl、Diag等距离
  double computeHFromCellToCell(int start_i, int start_j, int start_h,
                                int fin_i, int fin_j, int fin_h,
                                const EnvironmentOptions &options) {
    // return abs(fin_i - start_i) + abs(fin_j - start_j) + abs(fin_h -
    // start_h);

    if (options.metrictype == CN_SP_MT_MANH) {
      return options.linecost * (abs(fin_i - start_i) + abs(fin_j - start_j) +
                                 abs(fin_h - start_h));
    }
    if (options.metrictype == CN_SP_MT_CHEB) {
      return options.linecost *
             std::max(std::max(abs(fin_i - start_i), abs(fin_j - start_j)),
                      abs(fin_h - start_h));
    }
    if (options.metrictype == CN_SP_MT_EUCL) {
      return options.linecost * sqrt((fin_i - start_i) * (fin_i - start_i) +
                                     (fin_j - start_j) * (fin_j - start_j) +
                                     (fin_h - start_h) * (fin_h - start_h));
    }
    if (options.metrictype == CN_SP_MT_DIAG) {
      int d_i = abs(fin_i - start_i);
      int d_j = abs(fin_j - start_j);
      int d_h = abs(fin_h - start_h);
      int diag = std::min(std::min(d_i, d_j), d_h);
      d_i -= diag;
      d_j -= diag;
      d_h -= diag;
      if (d_i == 0) {
        return options.linecost * sqrt(3) * diag +
               options.diagonalcost * std::min(d_j, d_h) +
               options.linecost * abs(d_j - d_h);
      }
      if (d_j == 0) {
        return options.linecost * sqrt(3) * diag +
               options.diagonalcost * std::min(d_i, d_h) +
               options.linecost * abs(d_i - d_h);
      }
      if (d_h == 0) {
        return options.linecost * sqrt(3) * diag +
               options.diagonalcost * std::min(d_i, d_j) +  // NOLINT
               options.linecost * abs(d_i - d_j);
      }
    }
    return 0;
  }

  EnvironmentOptions options;
  SearchResult sresult;
  NodeList lppath, hppath;  // Found point by point and section paths
  std::unordered_map<uint_least32_t, node_t> close;
  std::unordered_map<uint_least32_t, node_t> *open;  // map数组
  std::vector<int_least64_t> openMinimums;

  octomap::IgTree *octree_ptr_;

  Map map;

  int openSize;
  float hweight, tweight, gweight;  // Heuristic weight coefficient
  int breakingties;  // ID of criterion which used for choosing between node
                     // with the same f-value

  ElevationMap elevation_map;  // too slow
  // std::vector<std::vector<int>> elevation_matrix; // too slow
  int *elevation_matrix;

  // void convertToElevationMap() {
  //   elevation_map.clear();
  //   auto start = octree_ptr_->begin_leafs();
  //   auto end = octree_ptr_->end_leafs();
  //   // 遍历八叉树节点，保存下采样点云用于kdtree
  //   for (auto iter = start; iter != end; iter++) {
  //     octomap::OcTreeKey iter_key = iter.getKey();
  //     octomap::IgTreeNode *iter_node = octree_ptr_->search(iter_key);
  //     if (!iter_node || !octree_ptr_->isNodeOccupied(iter_node)) continue;
  //     octomap::OcTreeKey key = iter_key;
  //     octomap::key_type z = key.k[2];
  //     key.k[2] = 0;
  //     if (elevation_map.count(key) == 0 || elevation_map[key] < z) {
  //       elevation_map[key] = z;
  //     }
  //   }
  // }

  // bool isInsideElevatioMap(int i, int j, octomap::OcTreeKey &key) {  //
  // NOLINT
  //   key = octree_ptr_->coordToKey(map.IndexToCoord(i, j, 0));
  //   auto k2d = key;
  //   k2d.k[2] = 0;
  //   auto iter = elevation_map.find(k2d);
  //   if (elevation_map.end() == iter) return false;
  //   key.k[3] = iter->second;
  //   return true;
  // }

  void convertToElevationMatrix() {
    // elevation_matrix.clear();  // too slow
    // //  i->height, j->width, z->altitude
    // elevation_matrix.resize(map.height);
    // for (auto row : elevation_matrix) {
    //   row.resize(map.width, -1e3);
    // }
    // elevation_map.clear();

    std::cout << "<GridAstarPlanner::convertToElevationMatrix>" << std::endl;
    auto start = octree_ptr_->begin_leafs();
    auto end = octree_ptr_->end_leafs();
    for (auto iter = start; iter != end; iter++) {
      octomap::OcTreeKey iter_key = iter.getKey();
      octomap::IgTreeNode *iter_node = octree_ptr_->search(iter_key);
      if (!iter_node || !octree_ptr_->isNodeOccupied(iter_node)) continue;
      auto p3d = octree_ptr_->keyToCoord(iter_key);
      int i, j, h;
      map.CoordToIndex(p3d, i, j, h);
      elevation_matrix[i * map.width + j] =
          std::max(elevation_matrix[i * map.width + j], h);  // NOLINT
    }
  }

  bool isInsideElevatioMatrix(int i, int j, int &h,       // NOLINT
                              octomap::OcTreeKey &key) {  // NOLINT
    if (i >= map.height || i < 0) return false;
    if (j >= map.width || j < 0) return false;
    h = elevation_matrix[i * map.width + j];
    if (h < -1e2) return false;
    key = octree_ptr_->coordToKey(map.IndexToCoord(i, j, h));
    return true;
  }
};

#endif  // ASTAR_3D_ASTAR_3D_H_
