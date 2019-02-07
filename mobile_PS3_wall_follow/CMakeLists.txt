cmake_minimum_required(VERSION 2.8.3)
project(mobile_PS3_wall_follow)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(wall_follow src/wall_follower.cpp)

cs_install()
cs_export()

