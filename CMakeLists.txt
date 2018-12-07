cmake_minimum_required(VERSION 2.8.3)
project(istiaslam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy geometry_msgs sensor_msgs nav_msgs tf
    )

find_package(Eigen3 REQUIRED)


catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES generic_slam_2d
    #  CATKIN_DEPENDS roscpp rospy std_msgs
    # DEPENDS opencv isam
    )

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    )

## Declare a C++ executable
add_executable(istia_slam_test_node istia_slam_test_node.cpp
    src/IstiaSlam_Costmap.cpp
    src/IstiaSlam_Map.cpp
    src/IstiaSlam.cpp
    )
## Declare a C++ executable
add_executable(istia_slam_cerema_node istia_slam_cerema_node.cpp
    src/IstiaSlam_Costmap.cpp
    src/IstiaSlam_Map.cpp
    src/IstiaSlam.cpp
    )

## Declare a C++ executable
add_executable(istia_slam_real_time_node istia_slam_real_time_node.cpp
    src/IstiaSlam_Costmap.cpp
    src/IstiaSlam_Map.cpp
    src/IstiaSlam.cpp
    )


## Specify libraries to link a library or executable target against
###          !!! IMPORTANT !!!
target_link_libraries(istia_slam_test_node
    ${catkin_LIBRARIES}
    )

target_link_libraries(istia_slam_cerema_node
    ${catkin_LIBRARIES}
    )

target_link_libraries(istia_slam_real_time_node
    ${catkin_LIBRARIES}
    )

