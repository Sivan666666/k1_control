cmake_minimum_required(VERSION 3.7.2)
project(jaka_ros_driver)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED true)

# 设置安全路径变量（避免特殊字符问题）
set(JAKA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/c&c++")  # 使用完整路径处理特殊字符

# 定义头文件目录 - 使用完整路径并包含在引号中
set(SDK_INCLUDE_DIR "${JAKA_DIR}/inc_of_c++")

# 定义库文件目录 - 使用完整路径
set(SDK_LIB_DIR "${JAKA_DIR}/x86_64-linux-gnu/shared/libjakaAPI_2_3_0_12.so")

# 调试输出路径
message(STATUS "------------------------------------------------------")
message(STATUS "JAKA_DIR: ${JAKA_DIR}")
message(STATUS "SDK_INCLUDE_DIR: ${SDK_INCLUDE_DIR}")
message(STATUS "SDK_LIB_DIR: ${SDK_LIB_DIR}")
message(STATUS "------------------------------------------------------")

# 检查头文件是否存在
if(EXISTS "${SDK_INCLUDE_DIR}/JAKAZuRobot.h")
    message(STATUS "✅ Found JAKAZuRobot.h!")
else()
    message(FATAL_ERROR "❌ JAKAZuRobot.h not found! Path: ${SDK_INCLUDE_DIR}/JAKAZuRobot.h")
endif()

# 检查库文件是否存在
if(EXISTS "${SDK_LIB_DIR}")
    message(STATUS "✅ Found JAKA SDK library!")
else()
    message(FATAL_ERROR "❌ JAKA SDK library not found! Path: ${SDK_LIB_DIR}")
endif()

# 添加ROS依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  trajectory_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs   # 根据实际使用的消息类型添加
  geometry_msgs
)

# 设置包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  "${SDK_INCLUDE_DIR}"  # 确保路径在引号中
)

# 设置链接库
set(COMMON_LIBS 
  "${SDK_LIB_DIR}"  # 使用完整路径在引号中
  pthread 
  ${catkin_LIBRARIES}
)

# ROS包配置
catkin_package()

# 添加控制执行节点
add_executable(arm_control_node
  src/arm_control_node.cpp src/common.h src/timespec.cpp src/common.cpp
)
target_link_libraries(arm_control_node
  ${catkin_LIBRARIES}
  ${COMMON_LIBS} # 链接JAKA SDK库
)
add_dependencies(arm_control_node
  ${catkin_EXPORTED_TARGETS}  # 替代${PROJECT_NAME}_generate_messages_cpp
)

# 添加指令生成节点
add_executable(command_publisher_node
  src/command_publisher_node.cpp
)
target_link_libraries(command_publisher_node
  ${catkin_LIBRARIES}
)


add_executable(sub src/sub.cpp src/common.h src/timespec.cpp src/common.cpp)
target_link_libraries(sub ${catkin_LIBRARIES} ${COMMON_LIBS})
add_dependencies(sub
  ${catkin_EXPORTED_TARGETS}  # 替代${PROJECT_NAME}_generate_messages_cpp
)
