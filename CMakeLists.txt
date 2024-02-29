cmake_minimum_required(VERSION 3.0.2)
project(laser_uav_estimator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  std_msgs
  mrs_msgs
  sensor_msgs
  geometry_msgs
  roscpp
  mrs_lib
)

# find Eigen 
find_package(Eigen3 REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES HeightEstimator
 CATKIN_DEPENDS nodelet  roscpp std_msgs mrs_lib mrs_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(HeightEstimator src/height_estimator.cpp)
add_dependencies(HeightEstimator 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(HeightEstimator
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)
