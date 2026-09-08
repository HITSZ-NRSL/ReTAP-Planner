/*
 * Created on Wed May 12 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#pragma once

#include <functional>
#include <unordered_map>
#include <vector>

template <typename T, typename Hash = std::hash<T>>
class DisjointSet {
 public:
  void insert(const T& x) {
    if (roots_.count(x) != 0) {
      return;
    }
    roots_[x] = x;
  }

  T find(const T& x) {
    if (roots_[x] == x) {
      return x;
    } else {
      roots_[x] = find(roots_[x]);
      return roots_[x];
    }
  }

  void merge(const T& lhs, const T& rhs) {
    T lhs_root = find(lhs);
    T rhs_root = find(rhs);
    if (lhs_root == rhs_root) {
      return;
    }
    if (rank_[lhs_root] <= rank_[rhs_root]) {
      roots_[lhs_root] = rhs_root;
    } else {
      roots_[rhs_root] = lhs_root;
    }
    if (rank_[lhs_root] == rank_[rhs_root] && lhs_root != rhs_root) {
      ++rank_[rhs_root];
    }
  }

  int count(T x) { return roots_.count(x); }

  std::unordered_map<T, T, Hash>& roots() { return roots_; }

  int getUnionedVoxelSize(const T& x) {
    T x_root = find(x);
    int res = 0;
    for (auto voxel : roots_) {
      if (find(voxel.second) == x_root) {
        ++res;
      }
    }
    return res;
  }

  std::vector<T> getUnionedVoxels(const T& x) {  // work
    std::vector<T> vec;
    T x_root = find(x);
    int res = 0;
    for (auto voxel : roots_) {
      if (find(voxel.second) == x_root) {
        vec.push_back(voxel.first);
      }
    }
    return vec;
  }

 private:
  std::unordered_map<T, T, Hash> roots_;
  /* uint16_t is sufficient for octomap */
  std::unordered_map<T, uint16_t, Hash> rank_;
};
