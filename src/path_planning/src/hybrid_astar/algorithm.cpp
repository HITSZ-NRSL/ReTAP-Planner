#include "hybrid_astar/algorithm.h"

#include <omp.h>

#include <boost/heap/binomial_heap.hpp>
#include <mutex>

using namespace HybridAStar;  // NOLINT

// ###################################################
//                                     NODE COMPARISON
// ###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

// ###################################################
//                                         2D A*
// ###################################################
float aStar(Node2D& start, Node2D& goal, Node2D* nodes2D, int width,  // NOLINT
            int height, CollisionDetection& configurationSpace) {     // NOLINT
  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // reset the open and closed list
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  // VISUALIZATION DELAY
  ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D*, boost::heap::compare<CompareNodes>> O;
  // update h value
  start.updateH(goal);
  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);
  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  // NODE POINTER
  Node2D* nPred;
  Node2D* nSucc;

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width);

    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    } else if (nodes2D[iPred].isOpen()) {  // EXPANSION OF NODE
      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      // remove node from open list
      O.pop();

      // GOAL TEST
      if (*nPred == goal) {
        return nPred->getG();
      } else {  // CONTINUE WITH SEARCH
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < Node2D::dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (nSucc->isOnGrid(width, height) &&
              configurationSpace.isTraversable(nSucc) &&
              !nodes2D[iSucc].isClosed()) {
            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it
            // on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              // calculate the H value
              nSucc->updateH(goal);
              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else {
              delete nSucc;
            }
          } else {
            delete nSucc;
          }
        }
      }
    }
  }

  // return large number to guide search away
  return 1000;
}

// ###################################################
//                                          COST TO GO
// ###################################################
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D,  // NOLINT
             float* dubinsLookup, int width, int height,
             CollisionDetection& configurationSpace) {  // NOLINT
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (Constants::dubins) {  // unused
    // ONLY FOR dubinsLookup
    ompl::base::DubinsStateSpace dubinsPath(Constants::r);
    State* dbStart = (State*)dubinsPath.allocState();  // NOLINT
    State* dbEnd = (State*)dubinsPath.allocState();    // NOLINT
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
  }

  // if reversing is active use a, calculate reedsSheppPath and its cost
  if (Constants::reverse && !Constants::dubins) {
    //    ros::Time t0 = ros::Time::now();
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State* rsStart = (State*)reedsSheppPath.allocState();  // NOLINT
    State* rsEnd = (State*)reedsSheppPath.allocState();    // NOLINT
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 <<
    //    std::endl;
  }

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  int idx = (int)start.getY() * width + (int)start.getX();  // NOLINT
  if (Constants::twoD && !nodes2D[idx].isDiscovered()) {
    //    ros::Time t0 = ros::Time::now();
    // create a 2d start node
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    nodes2D[idx].setG(
        aStar(goal2d, start2d, nodes2D, width, height, configurationSpace));
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated 2D Heuristic in ms: " << d * 1000 <<
    //    std::endl;
  }

  if (Constants::twoD) {
    // offset for same node in cell
    auto delta_start_x = start.getX() - (long)start.getX();  // NOLINT
    auto delta_start_y = start.getY() - (long)start.getY();  // NOLINT
    auto delta_goal_x = goal.getX() - (long)goal.getX();     // NOLINT
    auto delta_goal_y = goal.getY() - (long)goal.getY();     // NOLINT

    twoDoffset = sqrt(pow(delta_start_x - delta_goal_x, 2) +
                      pow(delta_start_y - delta_goal_y, 2));

    twoDCost = nodes2D[idx].getG() - twoDoffset;
  }

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
}

// ###################################################
//                                         DUBINS SHOT
// ###################################################
Node3D* dubinsShot(Node3D& start, const Node3D& goal,         // NOLINT
                   CollisionDetection& configurationSpace) {  // NOLINT
  double q0[] = {start.getX(), start.getY(), start.getT()};   // start
  double q1[] = {goal.getX(), goal.getY(), goal.getT()};      // goal

  DubinsPath path;
  dubins_init(q0, q1, Constants::r, &path);  // calculate the path

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  Node3D* dubinsNodes =
      new Node3D[(int)(length / Constants::dubinsStepSize) + 1];  // NOLINT

  // Node3D* ret = new Node3D();

  x += Constants::dubinsStepSize;  // avoid duplicate waypoint
  int cnt = 0;
  while (x < length) {
    std::cout << "<dubinsShot> cnt=" << cnt++ << ", x=" << x
              << ", length=" << length << std::endl;
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    if (configurationSpace.isTraversable(&dubinsNodes[i])) {  // collision check
      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        std::cout << "looping shot";
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {
      // std::cout << "Dubins shot collided, discarding the path" << "\n";
      delete[] dubinsNodes;  // delete all nodes
      return nullptr;
    }
  }

  // *ret = dubinsNodes[i - 1];
  // delete[] dubinsNodes;  // delete all nodes
  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  return &dubinsNodes[i - 1];
}

// ###################################################
//                                         3D A*
// ###################################################
Node3D* Algorithm::hybridAStar(Node3D& start, const Node3D& goal,
                               Node3D* nodes3D, Node2D* nodes2D, int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup) {
  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  int dir = Constants::reverse ? 6 : 3;  // Number of possible directions, 3 for
                                         // forward driving and an additional 3
                                         // for reversing
  int iterations = 0;  // Number of iterations the algorithm has run for
                       // stopping based on Constants::iterations

  boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>
      O;  // OPEN LIST AS BOOST IMPLEMENTATION(priorityQueue)

  omp_set_num_threads(6);
  updateH(start, goal, nodes2D, dubinsLookup, width, height,
          configurationSpace);  // update h value of the start point
  start.open();                 // mark start as open
  O.push(&start);               // push on priority queue aka open list
  iPred = start.setIdx(width, height);
  nodes3D[iPred] = start;

  Node3D* nPred;
  Node3D* nSucc;  // NODE POINTER

  int cnt = 0;
  static omp_lock_t lock;
  while (!O.empty()) {  // continue until O empty
    nPred = O.top();    // pop node with lowest cost from priority queue
    iPred = nPred->setIdx(width, height);  // set index
    iterations++;

    // LAZY DELETION of rewired node
    if (nodes3D[iPred].isClosed()) {  // if there exists a pointer this node has
                                      // already been expanded
      O.pop();  // pop node from the open list and start with a fresh node
      continue;
    }

    if (!nodes3D[iPred].isOpen()) {
      continue;
    }

    // EXPANSION OF A OPEN NODE
    nodes3D[iPred].close();  // add node to closed list
    O.pop();                 // remove node from open list

    if (*nPred == goal) {
      std::cout << "<hybridAStar>: Reach goal." << std::endl;
      return nPred;
    }

    // CONTINUE SEARCH WITH DUBINS SHOT
    if (Constants::dubinsShot && nPred->isInRange(goal) &&
        nPred->getPrim() < 3) {
      nSucc = dubinsShot(*nPred, goal, configurationSpace);

      // 只有在到达终点后返回有效的节点
      if (nSucc != nullptr && *nSucc == goal) {
        printf("<dubinsShot>: reach goal (%.3f,%.3f) @ (%.3f,%.3f)\n",
               goal.getX(), goal.getY(), nPred->getX(),
               nPred->getY());  // DEBUG
        return nSucc;
      }
    }

    omp_init_lock(&lock);  // 初始化互斥锁
    ros::WallTime t = ros::WallTime::now();
// SEARCH WITH FORWARD SIMULATION
#pragma omp parallel for
    for (int i = 0; i < dir; i++) {
      Node3D* node = nPred->createSuccessor(i);   // create possible successor
      int succ_id = node->setIdx(width, height);  // set index of the successor

      // ensure successor is on grid and traversable
      if (!node->isOnGrid(width, height) ||
          !configurationSpace.isTraversable(node)) {
        delete node;
        continue;
      }
      omp_set_lock(&lock);  // 获得互斥器
      cnt++;
      // ensure successor is not on closed list or
      // it has the same index as the predecessor
      // if (!nodes3D[succ_id].isClosed() || iPred == succ_id) {
      if (nodes3D[succ_id].isClosed() && iPred != succ_id) {
        delete node;
        omp_unset_lock(&lock);  // 释放互斥器
        continue;
      }

      // calculate new G value
      node->updateG();
      newG = node->getG();

      // if successor not on open list or found a shorter way to the cell
      // if (!nodes3D[succ_id].isOpen() || newG < nodes3D[succ_id].getG() ||
      //     iPred == succ_id)
      if (nodes3D[succ_id].isOpen() && newG >= nodes3D[succ_id].getG() &&
          iPred != succ_id) {
        delete node;
        omp_unset_lock(&lock);  // 释放互斥器
        continue;
      }

      // calculate H value
      updateH(*node, goal, nodes2D, dubinsLookup, width, height,
              configurationSpace);

      // if the successor is in the same cell but the C value is larger
      if (iPred == succ_id &&
          node->getC() > nPred->getC() + Constants::tieBreaker) {
        delete node;
        omp_unset_lock(&lock);  // 释放互斥器
        continue;
      }

      // if successor is in the same cell and the C value is lower,
      // set predecessor to predecessor of predecessor
      if (iPred == succ_id && nPred->getPred() != nullptr &&
          node->getC() <= nPred->getC() + Constants::tieBreaker) {
        printf("Set predecessor\n");
        printf(
            "Set predecessor to predecessor of predecessor: -------\n "
            "\tpred_pred(%.3f,%.3f), pred(%.3f,%.3f), node(%.3f,%.3f)\n",
            nPred->getPred()->getX(), nPred->getPred()->getY(), nPred->getX(),
            nPred->getY(), node->getX(), node->getY());
        float dis = sqrt(pow(node->getX() - node->getPred()->getX(), 2) +
                         pow(node->getY() - node->getPred()->getY(), 2));
        if (dis > 0.8) {  // add by liyx
          node->setMid(nPred);
          printf("\tdis: %.3f > 1, set mid(%.3f,%.3f,%.3f|%.3f,%.3f)\n", dis,
                 node->getMid().x, node->getMid().y, node->getMid().z,
                 node->getMid().ff, node->getMid().rf);
        } else {
          printf("\tdis: %.3f < 1\n", dis);
        }
        node->setPred(nPred->getPred());
      }

      if (node->getPred() == node) {
        std::cout << "looping\n";
      }

      node->open();  // put successor on open list
      nodes3D[succ_id] = *node;
      O.push(&nodes3D[succ_id]);
      delete node;
      omp_unset_lock(&lock);  // 释放互斥器
    }

    omp_destroy_lock(&lock);  // 销毁互斥器
    ros::WallDuration d = ros::WallTime::now() - t;
    printf(
        "<hybridAStar>: %d/%d-th iteration, %d-th checked node, "
        "timecost=%.1fms\n",
        iterations, Constants::iterations, cnt, d.toSec() * 1000);
    if (iterations > Constants::iterations) {
      ROS_ERROR(
          "<hybridAStar>: iteration timecount is out of bound. return "
          "nullptr.");
      // return nPred;
      break;
    }
  }

  if (O.empty()) {  // Openset为空了还没返回，说明没有有效路径
    ROS_ERROR("<hybridAStar>: Openset is empty. return nullptr.");
    return nullptr;
  }

  return nullptr;
}
