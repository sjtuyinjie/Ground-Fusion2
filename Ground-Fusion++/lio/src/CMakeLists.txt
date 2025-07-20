# 收集 src 目录下的所有源文件
file(GLOB_RECURSE LIO_SOURCES
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*.cc
    ${PROJECT_SOURCE_DIR}/src/*.c
)

# 创建静态库或动态库
add_library(lio_lib STATIC ${LIO_SOURCES})  # 如果需要动态库，将 STATIC 替换为 SHARED

# 链接依赖库
target_link_libraries(lio_lib
    ${catkin_LIBRARIES}
    ${g2o_libs}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    glog gflags
    yaml-cpp
    robin_map
)

add_subdirectory(common)

add_subdirectory(tools)

add_subdirectory(preprocess)

add_subdirectory(liw)

add_subdirectory(algo)

add_subdirectory(apps)