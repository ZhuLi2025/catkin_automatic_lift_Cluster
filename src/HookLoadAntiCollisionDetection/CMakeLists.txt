cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(HookLoadAntiCollisionDetection VERSION 1.0.0)

# 1. 编译选项设置
add_compile_options(-Wall -std=c++14 -O3)

add_definitions(-DCONFIG_FILE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# 2. 依赖查找
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
)

# 针对你记忆中 PCL 1.12 的自定义安装路径，建议在执行 cmake 时传入:
# -DPCL_DIR=/usr/local/lib/pcl-1.12/cmake
find_package(VTK 6.3.0 EXACT REQUIRED)
find_package(PCL 1.8.1 EXACT REQUIRED)
find_package(yaml-cpp REQUIRED)

# 3. 消息生成
add_message_files(FILES CraneStatus.msg)
generate_messages(DEPENDENCIES std_msgs)

# 4. 源码扫描
# 将核心算法源码与 main 函数入口分离
file(GLOB_RECURSE LIB_SOURCES 
    "src/processbackend.cpp"
    "src/pipeline.cpp"
    "src/HookLoadPositionAcquirer.cpp"
    "src/ICPTracker.cpp"
    "src/CollisionDetector.cpp"
    "src/CSF/*.cpp"
)

# 5. 定义动态链接库目标
add_library(${PROJECT_NAME} SHARED ${LIB_SOURCES})

# 6. 配置包含目录 (现代 CMake 方式)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

# 7. 链接依赖库
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE ROSBAG_TEST)

# 8. 如果仍需生成可执行节点，链接到上面的动态库
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# 9. Catkin 导出配置
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp std_msgs message_runtime
)

# 10. 安装与导出 (参照 zl_motion_planning 规范)
include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

# 生成并安装 CMake 配置文件，以便第三方非 ROS 项目 find_package
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(EXPORT ${PROJECT_NAME}-targets
    FILE ${PROJECT_NAME}-targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake
)
