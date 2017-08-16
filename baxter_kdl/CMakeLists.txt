cmake_minimum_required(VERSION 2.8.3)
project(baxter_kdl)

find_package(Eigen3 REQUIRED)
find_package(catkin
  REQUIRED
  COMPONENTS
  roscpp
  kdl_parser
  baxter_core_msgs
)

include_directories(
  include/${PROJECT_NAME}             
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  DEPENDS
  orocos_kdl
  kdl_parser
  baxter_core_msgs
  LIBRARIES baxter_kdl
  INCLUDE_DIRS include
)

add_library(baxter_kdl src/baxter_kdl.cpp)
target_link_libraries(baxter_kdl ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)



