# 设置最小 CMake 版本
cmake_minimum_required(VERSION 3.16)

# 设置项目名称和语言
project(MyProject LANGUAGES CXX CUDA)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 启用 CUDA 语言支持
enable_language(CUDA)

# 查找 OpenMP 包
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# 添加源文件
set(SOURCES
    src/main.cpp
    src/kernel.cu
)

# 添加可执行文件
add_executable(MyProject ${SOURCES})

# 设置 CUDA 编译选项（可选）
set_target_properties(MyProject PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_ARCHITECTURES "80"  # 根据你的 GPU 架构调整
)

# 链接 CUDA 库（如果需要）
find_package(CUDA REQUIRED)
target_link_libraries(MyProject ${CUDA_LIBRARIES})