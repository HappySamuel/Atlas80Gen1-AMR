cmake_minimum_required(VERSION 2.8.3)
project(laser_maxrange_republish)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laser_maxrange_republish
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(hokuyo_repub src/hokuyo_repub.cpp)
add_dependencies(hokuyo_repub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hokuyo_repub ${catkin_LIBRARIES})

add_executable(fov_narrower src/fov_narrower.cpp)
add_dependencies(fov_narrower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fov_narrower ${catkin_LIBRARIES})

add_executable(laser_repub src/laser_filter_republish.cpp)
add_dependencies(laser_repub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_repub ${catkin_LIBRARIES})


add_executable(laser_repub_no_intensity
    src/laser_filter_republish_no_intensity.cpp)
add_dependencies(laser_repub_no_intensity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_repub_no_intensity ${catkin_LIBRARIES})

