cmake_minimum_required(VERSION 3.0.2)
project(cilqr)

set(CMAKE_CXX_FLAGS "-O3" )
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_core
  roscpp
  rospy
  sensor_msgs
  std_msgs

  nav_msgs
  geometry_msgs
  ackermann_msgs
  visualization_msgs
  dynamic_reconfigure
  tf
  tf2_ros
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  message_generation

  Eigen3
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cilqr
#  CATKIN_DEPENDS nav_core roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime roscpp rospy sensor_msgs std_msgs nav_msgs geometry_msgs visualization_msgs nav_core
)

include_directories(
# include
${catkin_INCLUDE_DIRS}
${eigen3_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}/unsupported
/opt/intel/oneapi/mkl/2024.0/include/
)

link_directories(
  ${catkin_LIB_DIRS}
  /opt/intel/oneapi/mkl/latest/lib/
)
# file(GLOB SOURCES "include/cilqr/*.cpp" "include/ilqr/*.cpp")
# add_library(cilqr_planner ${SOURCES})
add_library(cilqr_planner include/cilqr/cilqr_planner.cpp include/cilqr/ilqr_uncertainty_node.cpp include/cilqr/Uncertainty.cpp include/cilqr/iLQR.cpp include/cilqr/LocalPlanner.cpp include/cilqr/Constraints.cpp include/cilqr/Model.cpp include/cilqr/Obstacle.cpp include/cilqr/Parameters.cpp)

# add_executable(${PROJECT_NAME}_node src/main.cpp include/ilqr/ilqr_uncertainty_node.cpp include/ilqr/Uncertainty.cpp include/ilqr/iLQR.cpp include/ilqr/LocalPlanner.cpp include/ilqr/Constraints.cpp include/ilqr/Model.cpp include/ilqr/Obstacle.cpp include/ilqr/Parameters.cpp)
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES} 
#   libmkl_rt.so
# )

target_link_libraries(cilqr_planner
  ${catkin_LIBRARIES} 
  libmkl_rt.so
)
