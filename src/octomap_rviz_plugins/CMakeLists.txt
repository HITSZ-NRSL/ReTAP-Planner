cmake_minimum_required(VERSION 3.0.2)
project(octomap_rviz_plugins)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

find_package(catkin REQUIRED COMPONENTS octomap_msgs
                                        roscpp
                                        rviz
                                        mapping_module
                                        backward_ros
)

find_package(octomap REQUIRED)
find_package(PCL REQUIRED)
find_package(mapping_module REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread )

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)
# set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -ldl -lbfd")
set(CMAKE_AUTOMOC ON)

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS octomap_msgs
                   roscpp
                   rviz
    DEPENDS OCTOMAP
)
                                        

# message("catkin_LIBRARIES:")
# message(${catkin_LIBRARIES})
include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${Boost_INCLUDE_DIRS} 
                    ${OCTOMAP_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    )

link_directories(${catkin_LIBRARY_DIRS}
                 ${Boost_LIBRARY_DIRS}
                 ${OCTOMAP_LIBRARY_DIRS}) 


set(SOURCE_FILES
  src/occupancy_grid_display.cpp
  src/occupancy_map_display.cpp 
)

set(HEADER_FILES
  include/octomap_rviz_plugins/occupancy_grid_display.h
  include/octomap_rviz_plugins/occupancy_map_display.h
)

add_library(${PROJECT_NAME} MODULE ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} 
${QT_LIBRARIES} 
${Boost_LIBRARIES} 
${OCTOMAP_LIBRARIES} 
${catkin_LIBRARIES}
${PCL_LIBRARIES}
mapping_module_world_representation
)
target_compile_options(${PROJECT_NAME} PUBLIC "-Wno-register") # Avoid OGRE deprecaton warnings under C++17

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


