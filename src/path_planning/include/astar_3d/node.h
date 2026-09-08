#ifndef node_t_H
#define node_t_H

#include <functional>
struct node_t {
  int i, j, z;
  double F;  // Weighted huristic cost
  double g;  // Accumulated Move cost
  double H;  // Accumulated huristic cost: 从current到goal的距离
  double T;  // Accumulated Traversability cost
  const node_t* parent;

  bool operator==(const node_t& other) const {
    return i == other.i && j == other.j && z == other.z;
  }
  bool operator!=(const node_t& other) const { return !operator==(other); }

  uint_least32_t get_id(int map_height, int map_width) {
    return (uint_least32_t)map_height * map_width * z + map_width * i + j;
  }
};

namespace std {
template <>
struct hash<node_t> {
  size_t operator()(const node_t& x) const;
};
}  // namespace std

size_t std::hash<node_t>::operator()(const node_t& x) const {
  size_t seed = 0;
  seed ^= std::hash<int>()(x.i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<int>()(x.j) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<int>()(x.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

  return seed;
}

#endif
