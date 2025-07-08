cmake_minimum_required(VERSION 3.7.2)
project(k1_control VERSION 1.0 LANGUAGES CXX)

# C++标准配置
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # 禁用编译器扩展（如GNU的变种C++）

# 平台相关库路径
if(UNIX AND NOT APPLE)
    set(SDK_LIB_DIR ${CMAKE_SOURCE_DIR}/libs/linux_x86_64)
elseif(WIN32)
    set(SDK_LIB_DIR ${CMAKE_SOURCE_DIR}/libs/windows_x64)
endif()

# 打印项目根目录（调试用）
message("Project root: ${CMAKE_SOURCE_DIR}")

# 查找库文件
find_library(JAKA_API 
    NAMES libjakaAPI_2_3_1_0.so libjakaAPI_2_3_1_0.dll
    PATHS ${SDK_LIB_DIR}
    REQUIRED
    NO_DEFAULT_PATH  # 禁止搜索系统路径
)

# 显式声明源文件
set(SOURCE_FILES
    src/main.cpp
    src/main2.cpp
    src/turn_cloth_demo.cpp
)

# 生成可执行文件
foreach(source_file ${SOURCE_FILES})
    get_filename_component(executable_name ${source_file} NAME_WE)
    add_executable(${executable_name} ${source_file})
    
    # 链接配置
    target_include_directories(${executable_name} PUBLIC 
        ${CMAKE_SOURCE_DIR}/include
    )
    target_link_libraries(${executable_name} 
        ${JAKA_API} 
        pthread
    )
    
    # 输出目录和编译选项
    set_target_properties(${executable_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
        OUTPUT_NAME "${executable_name}"  
    )
    
    # 安装规则
    install(TARGETS ${executable_name} DESTINATION bin)
    
    # 调试符号优化
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(${executable_name} PRIVATE -g -O0)
    else()
        target_compile_options(${executable_name} PRIVATE -O3)
    endif()
endforeach()

# 安装头文件
install(DIRECTORY include/ DESTINATION include/k1_control)