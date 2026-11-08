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
        controller_manager_msgs
        controller_interface
        hardware_interface
        pluginlib
        realtime_tools
        control_toolbox
        )

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
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

add_library(${PROJECT_NAME}
    src/hero_chassis_controller.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(
    TARGETS ${PROJECT_NAME}
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
      test/hero_chassis_controller.cpp)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif ()

roslint_cpp()
