cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint
        sensor_msgs
        geometry_msgs
        nav_msgs
        tf
        controller_manager_msgs
        controller_interface
        hardware_interface
        pluginlib
        realtime_tools
        control_toolbox
        dynamic_reconfigure
        )

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
    ${PROJECT_NAME}_core
    CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    controller_manager_msgs
    controller_interface
    hardware_interface
    pluginlib
    realtime_tools
    control_toolbox
    DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_core
    src/hero_chassis_controller.cpp
    src/pid.cpp
)
target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_11)

add_executable(${PROJECT_NAME}
    src/hero_chassis_controller_node.cpp
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

add_dependencies(${PROJECT_NAME}_core
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_core
    ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_core
    ${catkin_LIBRARIES}
)

install(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(
    FILES hero_chassis_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    DIRECTORY doc
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (${CATKIN_ENABLE_TESTING})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  catkin_add_gtest(${PROJECT_NAME}-test
      test/hero_chassis_controller.cpp
      test/pid.cpp)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
endif ()

roslint_cpp()
