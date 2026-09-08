cmake_minimum_required(VERSION 3.0.2)
project(path_planning)
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")

SET(qhullDir ${CMAKE_CURRENT_LIST_DIR}/lib/qhull)
SET(qhullInclude ${qhullDir}/src/libqhullcpp)
INCLUDE_DIRECTORIES(${qhullInclude})
INCLUDE_DIRECTORIES(${qhullDir}/src)
LINK_DIRECTORIES(${qhullDir}/lib)
LINK_DIRECTORIES(${qhullDir}/build)
SET(qhullLibs qhullcpp qhull_r)

find_package(catkin REQUIRED COMPONENTS octomap_msgs
                                        roscpp
                                        std_msgs
                                        tf
                                        tf_conversions
                                        rviz
                                        mapping_module
                                        backward_ros
                                        message_filters
)

find_package(octomap REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(ompl REQUIRED)  # add for hybrid astar
if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

find_package(OpenMP REQUIRED)  # add by liyx
if(OpenMP_FOUND)
    message(STATUS "found openmp")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
    message(FATAL_ERROR "openmp not found!")
endif()

###########
## Service ##
###########
add_service_files(
  FILES
  query_contact_configuration.srv
  query_line_contact_configuration.srv
  query_batch_line_contact_configuration.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include  ${Eigen_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS  message_runtime roscpp geometry_msgs visualization_msgs tf mapping_module
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenMP_CXX_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${qhullInclude}
)

###################################################
# Core: Local Planning Node (Iterative Geometric Evaluation)
###################################################
add_executable(local_planning_node
  src/local_planning/local_planning_node.cpp
  src/local_planning/local_planning.cpp
)

target_link_libraries(local_planning_node
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${qhullLibs}
  mapping_module_world_representation
)

###################################################
# Core: Global Planning Node (Terrain-Aware A*)
###################################################
add_executable(global_planning_node
  src/global_planning/global_planning_node.cpp
  src/global_planning/global_planning.cpp
)

target_link_libraries(global_planning_node
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  mapping_module_world_representation
)

###################################################
# Core: Hybrid A* Local Path Planning
###################################################
add_executable(hybrid_astar
    src/hybrid_astar/planner_node.cpp
    src/hybrid_astar/algorithm.cpp
    src/hybrid_astar/planner.cpp
    src/hybrid_astar/dubins.cpp
    src/local_planning/local_planning.cpp
)

target_link_libraries(hybrid_astar
    ${OMPL_LIBRARIES}
    ${OpenMP_CXX_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${qhullLibs}
)

###################################################
# Utility: Path Publisher
###################################################
add_executable(path_publisher_node
  src/apps/path_publisher_node.cpp
)

target_link_libraries(path_publisher_node
  ${catkin_LIBRARIES}
)

###################################################
# Utility: Map Publisher (publish octree from file)
###################################################
add_executable(map_publisher_node
  src/apps/map_publisher_node.cpp
)

target_link_libraries(map_publisher_node
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  mapping_module_world_representation
)

###################################################
# Utility: Map Saver (save local octree to file)
###################################################
add_executable(map_saver_node
  src/apps/map_saver_node.cpp
)

target_link_libraries(map_saver_node
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  mapping_module_world_representation
)

###################################################
# Utility: Waypoint Follower
###################################################
add_executable(waypoint_follower_node
  src/apps/waypoint_follower.cpp
)

target_link_libraries(waypoint_follower_node
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

###################################################
# Utility: Frame Merger (merge LiDAR frames)
###################################################
add_executable(frame_merger_node
  src/apps/frame_merger_node.cpp
)

target_link_libraries(frame_merger_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)