cmake_minimum_required(VERSION 2.8.3)
project(Lab9)

find_package(catkin_simple REQUIRED)
find_package(simple_baxter_gripper_interface REQUIRED)
find_package(baxter_fk_ik REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})

#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(baxter_trajectory_streamer src/baxter_trajectory_streamer.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
#cs_add_executable(trajectory_interpolator_as src/trajectory_interpolator_as.cpp)
cs_add_executable(left_arm_as src/left_arm_as.cpp)
cs_add_executable(rt_arm_as src/rt_arm_as.cpp)
cs_add_executable(lab9_Manip src/lab9_Manip.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
target_link_libraries(lab9_Manip baxter_trajectory_streamer simple_baxter_gripper_interface baxter_fk_ik)
cs_install()
cs_export()
    
