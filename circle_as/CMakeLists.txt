cmake_minimum_required(VERSION 2.8.3)
project(screw_loose)

find_package(catkin_simple REQUIRED)
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
catkin_simple()

# Executables

cs_add_executable(circle_action_client src/circle_action_client.cpp)
cs_add_executable(perception_server src/perception_server.cpp)
cs_install()
cs_export()
    
