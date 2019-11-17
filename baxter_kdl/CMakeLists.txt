cmake_minimum_required(VERSION 2.8.3)
project(baxter_kdl)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(catkin
  REQUIRED
  COMPONENTS
  roscpp
  kdl_parser
  baxter_core_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  DEPENDS
  orocos_kdl
  kdl_parser
  baxter_core_msgs
  # LIBRARIES baxter_kdl
  # INCLUDE_DIRS include
)

add_library(baxter_kdl src/baxter_kdl.cpp)
target_link_libraries(baxter_kdl ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(baxter_kdl_example 
                src/baxter_kdl.cpp
                src/baxter_kdl_example.cpp
              )
target_link_libraries(baxter_kdl_example  ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})


# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )



