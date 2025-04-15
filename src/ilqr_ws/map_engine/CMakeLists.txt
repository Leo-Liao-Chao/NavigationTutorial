cmake_minimum_required(VERSION 3.0.2)
project(map_engine)
enable_language(CUDA)  # 激活 CUDA 语言支持

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-fopenmp)
set(CMAKE_BUILD_TYPE "release")

# 设置语言标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CUDA_STANDARD 11)

find_package(CUDA REQUIRED)
find_package (OpenMP )
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
  dynamic_reconfigure
  grid_map_core
  grid_map_ros
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_rviz_plugin
  grid_map_visualization
  message_generation
  message_runtime
)

# 添加项目自身的库和依赖
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)  # 设置编译后的程序存放位置
aux_source_directory(src SRC_LIST)  # 添加 src 目录下的所有源文件
include_directories(include)  # 添加头文件路径

# 分别编译 .cu 文件和 .cpp 文件
set(CUDA_SOURCES
  src/arbitrary_transformation.cu
)

set(CPP_SOURCES
  src/local_costmap.cpp
)

# 创建一个自定义目标来编译 .cu 文件
cuda_add_library(cu_lib ${CUDA_SOURCES})

# 创建可执行文件并链接 CUDA 库
add_executable(local_costmap ${CPP_SOURCES})

set_target_properties ( local_costmap
    PROPERTIES
    CUDA_ARCHITECTURES "35;50;52;72;75"
    CUDA_SEPARABLE_COMPILATION ON
)

target_compile_features ( local_costmap 
    PUBLIC 
    cxx_std_11
)

set_target_properties ( local_costmap
    PROPERTIES
    CUDA_ARCHITECTURES "35;50;52;72;75"
    CUDA_SEPARABLE_COMPILATION ON
)

target_sources(local_costmap PRIVATE ${CPP_SOURCES} ${CUDA_SOURCES})
target_compile_options(local_costmap PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${OpenMP_CXX_FLAGS}>)

set(CUB_DIR /usr/local/cuda-11.4/targets/x86_64-linux/include/cub/cmake)
set(Thrust_DIR /usr/local/cuda-11.4/include/thrust/cmake)
find_package(Thrust REQUIRED CONFIG)
thrust_create_target(Thrust)

target_link_libraries(local_costmap
  PRIVATE
  ${OpenMP_CXX_LIBRARIES}
  ${catkin_LIBRARIES}
  Thrust
  cu_lib
  # -fopenmp
  
)

# 配置 Thrust
# set(CUB_DIR /usr/local/cuda-11.4/targets/x86_64-linux/include/cub/cmake)
# set(Thrust_DIR /usr/local/cuda-11.4/include/thrust/cmake)
# find_package(Thrust REQUIRED CONFIG)
# thrust_create_target(Thrust)
# target_link_libraries(local_costmap Thrust)

# 消息和动态配置
add_message_files(
  FILES
  map_param.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/map_engine.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_engine
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs grid_map
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)