cmake_minimum_required(VERSION 3.0.2)
project(ilqr)

set(CMAKE_CXX_FLAGS "-O3" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  ackermann_msgs
  # carla_msgs
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
  # vehiclepub 
)

find_package(Eigen3 REQUIRED)

# add_message_files(
#   FILES
#   Experiment.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime roscpp std_msgs nav_msgs geometry_msgs visualization_msgs
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


add_executable(${PROJECT_NAME}_node src/main.cpp include/ilqr/ilqr_uncertainty_node.cpp include/ilqr/Uncertainty.cpp include/ilqr/iLQR.cpp include/ilqr/LocalPlanner.cpp include/ilqr/Constraints.cpp include/ilqr/Model.cpp include/ilqr/Obstacle.cpp include/ilqr/Parameters.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} 
  libmkl_rt.so
)