cmake_minimum_required(VERSION 3.10)
project(hybrid_a_star)

message(STATUS "Hybrid A star")
message(STATUS "Author: Zhang Zhimeng")

#set(CMAKE_VERBOSE_MAKEFILE "true")

# Compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if (COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_FLAGS} doesn't have C++14 support.
                         Please use a different C++ compiler")
endif ()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

# Eigen
include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIR})

# glog
include(cmake/glog.cmake)
include_directories(${GLOG_INCLUDE_DIRS})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# catkin
find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    cv_bridge
    image_transport
    pcl_ros
    pcl_conversions
    nav_msgs
    tf
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_service_files(
    FILES
    mineServer.srv
 )

generate_messages(
    DEPENDENCIES
    std_msgs
    pcl_msgs
    sensor_msgs
)

catkin_package(
  DEPENDS PCL roscpp message_generation
)

add_library(
    PATH_SEARCHER_LIB SHARED
    src/rs_path.cpp
    src/costmap_subscriber.cpp
    src/goal_pose_subscriber.cpp
    src/init_pose_subscriber.cpp
    src/hybrid_a_star.cpp
    src/hybrid_a_star_flow.cpp
)

target_link_libraries(
    PATH_SEARCHER_LIB
    ${catkin_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(run_hybrid_astar app/run_hybrid_astar.cpp)
target_link_libraries(run_hybrid_astar PATH_SEARCHER_LIB)
add_dependencies(run_hybrid_astar hybrid_a_star_gencpp)
