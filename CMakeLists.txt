cmake_minimum_required(VERSION 3.0.2)
project(hero_chassis_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
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
    DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/hero_chassis_controller.cpp
#        src/keyboard_control.cpp
)

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)

add_executable(keyboard_control src/keyboard_control.cpp)

target_link_libraries(
        keyboard_control
        ${catkin_LIBRARIES}
)

target_link_libraries(
        hero_chassis_controller
        ${catkin_LIBRARIES}
)

install(TARGETS keyboard_control
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

roslint_cpp()