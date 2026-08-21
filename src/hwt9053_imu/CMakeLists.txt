cmake_minimum_required(VERSION 3.8)
project(hwt9053_imu)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(hwt9053_imu_node src/hwt9053_imu_node.cpp)
target_include_directories(hwt9053_imu_node PRIVATE include)
ament_target_dependencies(hwt9053_imu_node rclcpp sensor_msgs geometry_msgs std_srvs)

add_executable(imu_static_tf_node src/imu_static_tf_node.cpp)
ament_target_dependencies(imu_static_tf_node rclcpp geometry_msgs tf2_ros)

install(TARGETS hwt9053_imu_node imu_static_tf_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch scripts DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imu_math test/test_imu_math.cpp)
  target_include_directories(test_imu_math PRIVATE include)
endif()

ament_package()
