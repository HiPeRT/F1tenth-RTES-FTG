cmake_minimum_required(VERSION 3.5)
project(ftg)

add_compile_options(-std=c++14)

set(TKROS_VERSION $ENV{ROS_VERSION})
if(TKROS_VERSION)
    add_definitions(-DTKROS_VERSION=${TKROS_VERSION})
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
)

set(LINK_LIBRARIES
  yaml-cpp
)

#add_library(${PROJECT_NAME}
#  src/ftg_node.cpp
#)

add_library(cspline
  src/utils/CSpline.cpp
)
add_library(cspline2d
  src/utils/CSpline2D.cpp
)

add_executable(ftg_node
  src/ftg_node.cpp
  include/ftg_node.h
)

ament_target_dependencies(ftg_node rclcpp sensor_msgs std_msgs geometry_msgs ackermann_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs yaml-cpp)

target_link_libraries(ftg_node
  ${LINK_LIBRARIES}
	cspline2d
	cspline
)

install(TARGETS
  ftg_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  conf
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()
