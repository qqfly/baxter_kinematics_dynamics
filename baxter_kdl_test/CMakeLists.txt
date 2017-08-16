cmake_minimum_required(VERSION 2.8.3)
project(baxter_kdl_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  baxter_kdl
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS
  baxter_kdl
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(baxter_kdl_test src/baxter_kdl_test.cpp)
target_link_libraries(baxter_kdl_test ${catkin_LIBRARIES})
