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
find_package(visualization_msgs REQUIRED)

add_library(geometry_utils src/geometry_utils.cpp)
target_include_directories(geometry_utils PUBLIC include)

add_executable(shoulder_align_node src/shoulder_align_node.cpp)
target_include_directories(shoulder_align_node PRIVATE include)
target_link_libraries(shoulder_align_node geometry_utils)
ament_target_dependencies(shoulder_align_node geometry_msgs nav_msgs rclcpp std_msgs visualization_msgs)

install(TARGETS geometry_utils shoulder_align_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry_utils test/test_geometry_utils.cpp)
  target_include_directories(test_geometry_utils PRIVATE include)
  target_link_libraries(test_geometry_utils geometry_utils)
endif()

ament_package()
