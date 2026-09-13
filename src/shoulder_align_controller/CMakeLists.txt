cmake_minimum_required(VERSION 3.8)
project(shoulder_align_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(shoulder_align_node src/shoulder_align_node.cpp)
ament_target_dependencies(shoulder_align_node geometry_msgs nav_msgs rclcpp std_msgs)

install(TARGETS shoulder_align_node DESTINATION lib/${PROJECT_NAME})

ament_package()
