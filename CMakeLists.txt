cmake_minimum_required(VERSION 3.0.2)
project(hero_chassis_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries

find_package(catkin REQUIRED
        COMPONENTS
        catkin
        roscpp
        roslint
        pluginlib
        controller_interface
        hardware_interface
        control_toolbox
        geometry_msgs
        tf
        nav_msgs
)

catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    roslint
    pluginlib
    controller_interface
    hardware_interface
    control_toolbox
    geometry_msgs
    tf
    nav_msgs
    DEPENDS
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(keyboard_publisher src/keyboard_publisher.cpp)

add_library(${PROJECT_NAME}
        src/hero_chassis_controller.cpp
        src/keyboard_publisher.cpp
        include/hero_chassis_controller/hero_chassis_controller.h
)

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)


target_link_libraries(
        keyboard_publisher
        ${catkin_LIBRARIES}
)

target_link_libraries(
        hero_chassis_controller
        ${catkin_LIBRARIES}
)

install(TARGETS keyboard_publisher
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

roslint_cpp()