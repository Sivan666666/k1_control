cmake_minimum_required(VERSION 3.7.2)    # Minimum CMake Version 
project(sdk_test VERSION 1.0)     # Project Definition

# C++ Standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED true)

# Source Directory Information
message(${CMAKE_SOURCE_DIR})

# 定义通用的头文件目录
set(SDK_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/c&c++/inc_of_c++)

# 定义通用的库文件
set(SDK_LIB_DIR ${CMAKE_SOURCE_DIR}/c&c++/x86_64-linux-gnu/shared/libjakaAPI_2_3_1_0.so)
set(COMMON_LIBS ${SDK_LIB_DIR} pthread)

# 创建第一个可执行文件
add_executable(demo main.cpp)
target_include_directories(demo PUBLIC ${SDK_INCLUDE_DIR})
target_link_libraries(demo ${COMMON_LIBS})

# 创建第二个可执行文件
add_executable(move_fling main2.cpp)
target_include_directories(move_fling PUBLIC ${SDK_INCLUDE_DIR})
target_link_libraries(move_fling ${COMMON_LIBS})