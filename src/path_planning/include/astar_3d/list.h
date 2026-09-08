#ifndef LIST_H
#define LIST_H
#include <list>

#include "node.h"

class NodeList {
 public:
  NodeList(void) {}
  ~NodeList(void) {}

  bool find(int x, int y) {
    std::list<node_t>::iterator iter = List.end();
    for (; iter != List.begin(); --iter) {
      if ((iter->i == x) && (iter->j == y)) {
        return true;
      }
    }
    if ((iter->i == x) && (iter->j == y)) {
      return true;
    }
    return false;
  }

  std::list<node_t>::iterator find_i(int x, int y) {
    std::list<node_t>::iterator iter = List.begin();
    for (iter = List.begin(); iter != List.end(); ++iter) {
      if ((iter->i == x) && (iter->j == y)) {
        break;
      }
    }
    return iter;
  }

  std::list<node_t> List;
};
#endif  // ASTAR_3D_LIST_H_
