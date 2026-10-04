cmake_minimum_required(VERSION 3.8)
project(vehicle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(circle_driver src/circle_driver.cpp)
ament_target_dependencies(circle_driver rclcpp geometry_msgs)

add_executable(drive_distance src/drive_distance.cpp)
ament_target_dependencies(drive_distance rclcpp geometry_msgs nav_msgs)

install(TARGETS
  circle_driver
  drive_distance
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
