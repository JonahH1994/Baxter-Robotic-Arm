cmake_minimum_required(VERSION 2.8.3)
project(baxter_gazebo)

find_package(catkin
  REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    gazebo_ros_control
    controller_manager
    baxter_core_msgs
    std_msgs
    sensor_msgs
    Eigen3
)

find_package(octomap REQUIRED)
include_directories(
	include
${OCTOMAP_INCLUDE_DIRS})
#${OCCUPANCY_GRID_INCLUDE_DIRS}
#"${PROJECT_SOURCE_DIR}/include/occupancy_grid"
#)
#target_link_libraries(${OCTOMAP_LIBRARIES})

# Depend on system install of Gazebo
find_package(GAZEBO REQUIRED)
find_package(Eigen3 REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS
  baxter_core_msgs
  gazebo_ros_control
  roscpp
  controller_manager_msgs
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${BAXTER_GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(LINK_LIBS
  ${BAXTER_GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)
#add_executable(occupancy_grid src/occupancy_grid.cpp)
#target_link_libraries(occupancy_grid ${catkin_LIBRARIES})

add_executable(octomap_parser src/octomap_pasrser.cpp)
target_link_libraries(octomap_parser ${catkin_LIBRARIES})

add_library(baxter_gazebo_ros_control
  src/baxter_gazebo_ros_control_plugin.cpp
)

add_library(occupancy_grid src/occupancy_grid.cpp)
target_link_libraries(occupancy_grid ${LINK_LIBS})

target_link_libraries(baxter_gazebo_ros_control
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  gazebo_ros_control
)

add_dependencies(
  baxter_gazebo_ros_control
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  octomap_parser
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(octomap_parser occupancy_grid)

install(DIRECTORY include/"occupancy_grid"/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(
  TARGETS baxter_gazebo_ros_control
  occupancy_grid
  octomap_parser
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
