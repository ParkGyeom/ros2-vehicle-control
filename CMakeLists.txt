cmake_minimum_required(VERSION 3.8)
project(vehicle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_executable(circle_driver src/circle_driver.cpp)
ament_target_dependencies(circle_driver rclcpp geometry_msgs)

add_executable(drive_distance src/drive_distance.cpp)
ament_target_dependencies(drive_distance rclcpp geometry_msgs nav_msgs)

add_executable(obstacle_stop src/obstacle_stop.cpp)
ament_target_dependencies(obstacle_stop rclcpp geometry_msgs nav_msgs sensor_msgs vision_msgs)

install(TARGETS
  circle_driver
  drive_distance
  obstacle_stop
  DESTINATION lib/${PROJECT_NAME})

# Python 노드: ros2 run vehicle_control yolo_detector.py
install(PROGRAMS
  scripts/yolo_detector.py
  scripts/red_box_detector.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
