project(test)

cmake_minimum_required(VERSION 3.25)

set(CPP_EXAMPLE_SOURCES
    "23.multi_movj.cpp"
    "24.multi_movl.cpp"
    "25.kine.cpp"
    "26.motion_abort.cpp"
    "27.payload.cpp"
    "28.drag.cpp"
    "29.fct.cpp"
    "30.edgservo.cpp"
    "31.edgservo_fct.cpp"
    "40.error_and_clear.cpp"
    "41.collision.cpp"
    "42.enable.cpp"
    "43.tool_offset.cpp"
)

set(CMAKE_BUILD_TYPE Debug)

foreach(src ${CPP_EXAMPLE_SOURCES})
    # compile
    set(TMP_TGT_NAME "${src}.out")
    link_directories(${CMAKE_SOURCE_DIR}/out/shared)
    link_directories(${CMAKE_SOURCE_DIR}/out/shared/${CMAKE_BUILD_TYPE})
    add_executable(${TMP_TGT_NAME} ${src} common.h timespec.cpp common.cpp)
    target_include_directories(${TMP_TGT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/inc)
    # target_link_directories(${TMP_TGT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/out/shared)
    target_link_libraries(${TMP_TGT_NAME} PRIVATE jakaAPI_2_3_0_7 pthread)
    # install
    install(
        FILES ${src}
        DESTINATION ${JAKA_SDK_EXAMPLE_INSTALL_DIR}/c++
    )
endforeach()

install(
    FILES common.h
    DESTINATION ${JAKA_SDK_EXAMPLE_INSTALL_DIR}/c++
)

unset(CPP_EXAMPLE_SOURCES)
