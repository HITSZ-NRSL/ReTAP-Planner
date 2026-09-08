#ifndef NODE3D_H
#define NODE3D_H

#include <stdio.h>

#include <Eigen/Core>
#include <cmath>
#include <utility>

#include "hybrid_astar/constants.h"
#include "hybrid_astar/helper.h"
namespace HybridAStar {
/*!
   \brief A three dimensional node class that is at the heart of the algorithm.

   Each node has a unique configuration (x, y, theta) in the configuration space
   C.
*/
class MidInfo {
 public:
  bool flag = false;
  float x;
  float y;
  float z;
  float t;
  float ff;
  float rf;
};
class Node3D {
 public:
  /// The default constructor for 3D array initialization
  Node3D() : Node3D(0, 0, 0, 0, 0, 0, nullptr) {}
  /// Constructor for a node with the given arguments
  Node3D(float x, float y, float t, float g, float h, const Node3D* pred,
         int prim = 0) {
    this->x = x;
    this->y = y;
    this->t = t;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->idx = -1;
    this->prim = prim;
  }
  Node3D(float x, float y, float z, float t, float g, float h,
         const Node3D* pred, int prim = 0) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->t = t;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->idx = -1;
    this->prim = prim;
  }

  // GETTER METHODS
  /// get the x position
  float getX() const { return x; }
  /// get the y position
  float getY() const { return y; }
  /// get the z position
  float getZ() const { return z; }
  /// get the heading theta
  float getT() const { return t; }
  /// get the cost-so-far (real value)
  float getG() const { return g; }
  /// get the cost-to-come (heuristic value)
  float getH() const { return h; }
  /// get the total estimated cost
  float getC() const { return g + h; }
  /// get the index of the node in the 3D array
  int getIdx() const { return idx; }
  int getIdx2D() const { return idx_2d; }
  /// get the number associated with the motion primitive of the node
  int getPrim() const { return prim; }
  /// determine whether the node is open
  bool isOpen() const { return o; }
  /// determine whether the node is closed
  bool isClosed() const { return c; }
  /// determine whether the node is open
  const Node3D* getPred() const { return pred; }

  // SETTER METHODS
  /// set the x position
  void setX(const float& x) { this->x = x; }
  /// set the y position
  void setY(const float& y) { this->y = y; }
  /// set the z position
  void setZ(const float& z) { this->z = z; }
  /// set the heading theta
  void setT(const float& t) { this->t = t; }
  /// set the cost-so-far (real value)
  void setG(const float& g) { this->g = g; }
  /// set the cost-to-come (heuristic value)
  void setH(const float& h) { this->h = h; }
  /// set and get the index of the node in the 3D grid
  int setIdx(int width, int height) {
    this->idx_2d = (int)(y)*width + (int)(x);  // NOLINT
    this->idx =
        (int)(t / Constants::deltaHeadingRad) * width * height +  // NOLINT
        this->idx_2d;  // Constants::headings
    // printf("<setIdx>: width=%d, height=%d, depth=%d, t=%.3f, delta=%.3f\n",
    //        width, height, (int)(t / Constants::deltaHeadingRad), t,    //
    //        NOLINT Constants::deltaHeadingRad);
    return idx;
  }
  /// open the node
  void open() {
    o = true;
    c = false;
  }
  /// close the node
  void close() {
    c = true;
    o = false;
  }
  /// set a pointer to the predecessor of the node
  void setPred(const Node3D* pred) { this->pred = pred; }
  void setMid(const Node3D* mid) {
    this->mid.flag = true;
    this->mid.x = mid->getX();
    this->mid.y = mid->getY();
    this->mid.z = mid->getZ();
    this->mid.t = mid->getT();
    this->mid.ff = mid->getFlipperAngles().first;
    this->mid.rf = mid->getFlipperAngles().second;
  }
  MidInfo getMid() const { return mid; }
  float getOffsetZ() { return offset_z; }
  void setOffsetZ(float value) { offset_z = value; }
  Eigen::Quaternionf getPose() { return pose; }
  void setPose(Eigen::Quaternionf q) { pose = q; }

  // UPDATE METHODS
  /// Updates the cost-so-far for the node x' coming from its predecessor. It
  /// also discovers the node.
  void updateG() {
    // forward driving
    if (prim < 3) {
      // penalize turning
      if (pred->prim != prim) {
        // penalize change of direction
        if (pred->prim > 2) {
          g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;
        } else {
          g += dx[0] * Constants::penaltyTurning;
        }
      } else {
        g += dx[0];
      }
    } else {  // reverse driving
      // penalize turning and reversing
      if (pred->prim != prim) {
        // penalize change of direction
        if (pred->prim < 3) {
          g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing *
               Constants::penaltyCOD;
        } else {
          g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
        }
      } else {
        g += dx[0] * Constants::penaltyReversing;
      }
    }
  }

  // CUSTOM OPERATORS
  /// Custom operator to compare nodes. Nodes are equal if their x and y
  /// position as well as heading is similar.
  bool operator==(const Node3D& rhs) const {
    return (int)x == (int)rhs.x && (int)y == (int)rhs.y &&  // NOLINT
           (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
            std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
  }

  // RANGE CHECKING
  /// Determines whether it is appropriate to find a analytical solution.
  bool isInRange(const Node3D& goal) const {
    int random = rand() % 10 + 1;              // NOLINT
    float dx = std::abs(x - goal.x) / random;  //
    float dy = std::abs(y - goal.y) / random;  //
    return sqrt((dx * dx) + (dy * dy)) < Constants::dubinsShotDistance;
  }

  // GRID CHECKING
  /// Validity check to test, whether the node is in the 3D array.
  bool isOnGrid(const int width, const int height) const {
    return x >= 0 && x < width && y >= 0 && y < height &&
           (int)(t / Constants::deltaHeadingRad) >= 0 &&  // NOLINT
           (int)(t / Constants::deltaHeadingRad) <
               Constants::headings;  // NOLINT
  }

  // SUCCESSOR CREATION
  /// Creates a successor in the continous space.
  Node3D* createSuccessor(const int i) {
    float xSucc;
    float ySucc;
    float tSucc;

    // calculate successor positions forward
    if (i < 3) {
      xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
      ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
      tSucc = Helper::normalizeHeadingRad(t + dt[i]);
    } else {  // backwards
      xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
      ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);
      tSucc = Helper::normalizeHeadingRad(t - dt[i - 3]);
    }

    return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
  }

  // CONSTANT VALUES
  /// Number of possible directions
  static const int dir;
  /// Possible movements in the x direction
  static const float dx[];
  /// Possible movements in the y direction
  static const float dy[];
  /// Possible movements regarding heading theta
  static const float dt[];

  /// rad
  void setFlipperAngles(float front, float rear) {
    flipper_angles = std::make_pair(front, rear);
  }
  std::pair<float, float> getFlipperAngles() const { return flipper_angles; }

 private:
  /// the x position, decentralized cell index
  float x = 0;
  /// the y position, decentralized cell index
  float y = 0;
  /// the z position, decentralized cell index
  float z = 0;
  /// the heading theta
  float t;
  /// the cost-so-far
  float g;
  /// the cost-to-go
  float h;
  /// the index of the node in the 3D array
  int idx;
  int idx_2d;
  /// the open value
  bool o;
  /// the closed value
  bool c;
  /// the motion primitive of the node
  int prim;
  /// the predecessor pointer
  const Node3D* pred = nullptr;
  MidInfo mid;
  std::pair<float, float> flipper_angles;
  bool idx2d_flag = false;
  float offset_z = 0;
  Eigen::Quaternionf pose = Eigen::Quaternionf::Identity();
};

}  // namespace HybridAStar
#endif  // NODE3D_H
