cmake_minimum_required(VERSION 2.8.3)
project(turtle-test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(lab1 src/lab1.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(lab1  ${catkin_LIBRARIES} )
