cmake_minimum_required(VERSION 2.8.3)
project(octomap_world)

set(CMAKE_CXX_FLAGS "-Wall -g")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mapping_module     # for igtree
  backward_ros
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
find_package(PCL REQUIRED)
find_package(mapping_module REQUIRED)  # for igtree


###################################
## catkin specific configuration ##
###################################
catkin_package(
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


############
# BINARIES #
############

####### publish_loaded_octree.cpp #######
add_executable(publish_loaded_octree_node
src/publish_loaded_octree.cpp
)
target_link_libraries(publish_loaded_octree_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  mapping_module_world_representation
)

####### save_subscribed_octree.cpp #######
add_executable(save_subscribed_octree_node
src/save_subscribed_octree.cpp
)
target_link_libraries(save_subscribed_octree_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  mapping_module_world_representation
)