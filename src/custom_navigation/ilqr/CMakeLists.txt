cmake_minimum_required(VERSION 3.0.2)
project(ilqr)

set(CMAKE_CXX_FLAGS "-O3" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  Vehicle.msg
  VehicleArray.msg
)

# 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
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


add_executable(${PROJECT_NAME}_node src/main.cpp include/ilqr/iLQR.cpp include/ilqr/LocalPlanner.cpp include/ilqr/Constraints.cpp include/ilqr/Model.cpp include/ilqr/Obstacle.cpp include/ilqr/Parameters.cpp)
add_executable(${PROJECT_NAME}_vehicle_pub src/vehicle_pub.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} libmkl_rt.so
)

target_link_libraries(${PROJECT_NAME}_vehicle_pub 
  ${catkin_LIBRARIES}
)