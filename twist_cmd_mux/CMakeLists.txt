cmake_minimum_required(VERSION 2.8.3)
project(twist_cmd_mux)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nodelet
  pluginlib
  roscpp
  rospy
  std_msgs
)


## System dependencies are found with CMake's conventions
find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif()


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(cfg/reload.cfg)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES twist_cmd_mux
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs pluginlib nodelet roscpp rospy std_msgs
  DEPENDS yaml-cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME}_nodelet src/twist_cmd_mux_nodelet.cpp src/twist_cmd_subscribers.cpp)
#add_dependencies(${PROJECT_NAME}_nodelet twist_msgs)
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

