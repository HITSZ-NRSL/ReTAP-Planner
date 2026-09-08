cmake_minimum_required(VERSION 3.8)
# cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})

set(CMAKE_BUILD_TYPE "RELEASE")
# set(CMAKE_CXX_STANDARD 17)
# LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")
# set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -ldl -lbfd")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -O3 -march=native")
project(mapping_module LANGUAGES CXX)


######################### Packages #########################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  message_filters
  tf
  tf_conversions
  message_generation
  octomap_msgs
  octomap_ros
  backward_ros
  geometry_msgs
  visualization_msgs
  tf2
  genmsg 
  sensor_msgs	
  nav_msgs
)

find_package(octomap REQUIRED)
find_package(PCL REQUIRED)
find_package(cmake_modules REQUIRED)

###########
## Service ##
###########
add_service_files(
  FILES
  save_octomap.srv
  query_nearest_drivable_point.srv
  query_bbox_pcd.srv
  query_all_drivable_pcd.srv
  send_map_data.srv
  send_keyframe_data.srv
  query_terrain_attribute.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  octomap_msgs
  nav_msgs 
  visualization_msgs 
)

catkin_package(
 INCLUDE_DIRS include  ${Eigen_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} 
 LIBRARIES mapping_module  ${OCTOMAP_LIBRARIES} ${PROJECT_NAME}_world_representation  ${PROJECT_NAME}_utils 
 CATKIN_DEPENDS  message_runtime roscpp geometry_msgs visualization_msgs tf rospy  
)

set(Eigen_INCLUDE_DIRS "/usr/include/eigen3")
include_directories(
include
include/mapping_module
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

###########
## libs ##
###########

add_library(${PROJECT_NAME}_utils
  src/mapping_module/utils/octomap_utils.cpp
  src/mapping_module/utils/rviz_utils.cpp
  src/mapping_module/utils/bounding_box.cpp
  src/mapping_module/utils/math_utils.cpp
  src/mapping_module/utils/file_path.cpp
  src/mapping_module/utils/common_utils.cpp
)
target_link_libraries(${PROJECT_NAME}_utils
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)


add_library(${PROJECT_NAME}_world_representation
  src/mapping_module/world_representation/ig_tree.cpp
  src/mapping_module/world_representation/world_representation.cpp
)

target_link_libraries(${PROJECT_NAME}_world_representation
  ${OCTOMAP_LIBRARIES}
  ${PROJECT_NAME}_utils
  ${PCL_LIBRARIES}
)


add_library(${PROJECT_NAME}
src/mapping_module/mapping_module.cpp
src/mapping_module/terrain_evaluation.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  ${PROJECT_NAME}_world_representation
)

#############
##  Apps  ##
#############

########### mapping_module_node #########
add_executable(${PROJECT_NAME}_node
  src/apps/mapping_module_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME} 
)

########### mapping_module_node #########
add_executable(global_costmaper_node
  src/apps/global_costmaper_node.cpp
)

target_link_libraries(global_costmaper_node
  ${PROJECT_NAME} 
)

########### ghpr_test #########
# add_executable(ghpr_test
#   src/apps/ghpr_test.cpp
# )

# target_link_libraries(ghpr_test
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

# add_executable(wr_test_node
#   src/mapping_module/wr_test.cpp
# )

# target_link_libraries(wr_test_node
#   ${PROJECT_NAME} 
# )

####### odom_to_tf_node #######
add_executable(odom_to_tf_node
src/apps/odom_to_tf_node.cpp
)
target_link_libraries(odom_to_tf_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
)

####### publish_loaded_octree_std.cpp #######
add_executable(publish_loaded_octree_std_node
src/apps/publish_loaded_octree_std.cpp
)
target_link_libraries(publish_loaded_octree_std_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

####### send_relo_data_node.cpp #######
add_executable(send_relo_data_node
src/apps/send_relo_data_node.cpp
)
target_link_libraries(send_relo_data_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

###### map_merger_node ######
add_executable(map_merger_node
  src/apps/map_merger_node.cpp
)

target_link_libraries(map_merger_node
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  mapping_module_world_representation
)

###### pcd_smooth_mls_node ######
add_executable(pcd_smooth_mls_node
  src/apps/pcd_smooth_mls.cpp
)

target_link_libraries(pcd_smooth_mls_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


# ####### publish_loaded_octree.cpp #######
# add_executable(publish_loaded_octree_node
# src/apps/publish_loaded_octree.cpp
# )
# target_link_libraries(publish_loaded_octree_node
#   ${catkin_LIBRARIES}
#   ${OCTOMAP_LIBRARIES}
#   ${PROJECT_NAME}_world_representation
# )

# ####### save_advertised_octree.cpp #######
# add_executable(save_advertised_octree_node
# src/apps/save_advertised_octree.cpp
# )
# target_link_libraries(save_advertised_octree_node
#   ${catkin_LIBRARIES}
#   ${OCTOMAP_LIBRARIES}
#   ${PROJECT_NAME}_world_representation
# )


# install(
#   TARGETS   
#     ${PROJECT_NAME}_utils
#     ${PROJECT_NAME}_world_representation
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/mapping_module/world_representation
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   PATTERN ".svn" EXCLUDE)
