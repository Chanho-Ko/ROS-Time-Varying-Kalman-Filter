cmake_minimum_required(VERSION 3.0.2)
project(pos_est)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS} include)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs
  DEPENDS Eigen
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pos_est_node src/pos_est_node.cpp)

target_link_libraries(pos_est_node 
  ${catkin_LIBRARIES} 
  ${Eigen_LIBRARIES}
)
