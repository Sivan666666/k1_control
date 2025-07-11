cmake_minimum_required(VERSION 3.7.2)
project(k1_control VERSION 1.0 LANGUAGES CXX)

# C++标准配置
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 平台相关库路径
if(UNIX AND NOT APPLE)
    set(SDK_LIB_DIR ${CMAKE_SOURCE_DIR}/libs/linux_x86_64)
    # 添加数学库链接
    set(EXTRA_LIBS m)
    # 添加对POSIX时间的支持
    add_definitions(-D_POSIX_C_SOURCE=199309L)
elseif(WIN32)
    set(SDK_LIB_DIR ${CMAKE_SOURCE_DIR}/libs/windows_x64)
    set(EXTRA_LIBS "")
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

# 打印项目根目录（调试用）
message("Project root: ${CMAKE_SOURCE_DIR}")
message("SDK library directory: ${SDK_LIB_DIR}")

# 查找库文件 - 修正库文件名
find_library(JAKA_API 
    NAMES libjakaAPI_2_3_0_12.so  # 确保名称与实际文件名完全一致
    PATHS ${SDK_LIB_DIR}
    REQUIRED
    NO_DEFAULT_PATH
)

# 验证库文件是否找到
if(NOT JAKA_API)
    message(FATAL_ERROR "JAKA API library not found in ${SDK_LIB_DIR}")
else()
    message(STATUS "Found JAKA API library: ${JAKA_API}")
endif()

# 添加公共源文件
set(COMMON_SOURCES src/timespec.cpp)

# 显式声明源文件
set(SOURCE_FILES
    src/main.cpp
    src/main2.cpp
    src/turn_cloth_demo.cpp
    src/turn_circle.cpp
)

# 设置头文件搜索路径
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/inc_of_c    # 添加C头文件目录
)

# 生成可执行文件
foreach(source_file ${SOURCE_FILES})
    get_filename_component(executable_name ${source_file} NAME_WE)
    
    # 添加可执行文件
    add_executable(${executable_name} 
        ${source_file}
        ${COMMON_SOURCES}
    )
    
    # 链接配置
    target_link_libraries(${executable_name} 
        ${JAKA_API}
        pthread
        ${EXTRA_LIBS}  # 链接数学库
    )
    
    # 输出设置
    set_target_properties(${executable_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        OUTPUT_NAME "${executable_name}"
    )
    
    # 安装规则
    install(TARGETS ${executable_name} 
        RUNTIME DESTINATION bin
    )
    
    # 编译选项
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(${executable_name} PRIVATE -g -O0)
    else()
        target_compile_options(${executable_name} PRIVATE -O3)
    endif()
endforeach()

# 安装头文件
install(DIRECTORY include/ DESTINATION include/k1_control)
install(DIRECTORY libs/inc_of_cpp/ DESTINATION include/k1_control/inc_of_cpp)
install(DIRECTORY libs/inc_of_c/ DESTINATION include/k1_control/inc_of_c)

# 安装库文件
install(FILES ${SDK_LIB_DIR}/libjakaAPI_2_3_0_12.so
    DESTINATION lib
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)