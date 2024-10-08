cmake_minimum_required(VERSION 3.8)
project(labclass)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(lab1 src/lab1.cpp)
add_executable(lab2 src/lab2.cpp)
add_executable(lab3 src/lab3.cpp)
add_executable(lab4 src/lab4.cpp)
add_executable(scan_to_image_node_students src/scan_to_image_node_students.cpp)
add_executable(lab5 src/lab5.cpp)
add_executable(lab7 src/lab7.cpp)

ament_target_dependencies(lab1
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

ament_target_dependencies(lab2
  rclcpp
  sensor_msgs
)

ament_target_dependencies(lab3
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(lab4
  rclcpp
  sensor_msgs
  geometry_msgs
  OpenCV
)

ament_target_dependencies(scan_to_image_node_students
  rclcpp
  sensor_msgs
  geometry_msgs
  OpenCV
  nav_msgs
)

ament_target_dependencies(lab5
  rclcpp
  nav_msgs
  sensor_msgs
  OpenCV
  geometry_msgs
)


ament_target_dependencies(lab7
  rclcpp
  nav_msgs
  sensor_msgs
  OpenCV
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)
include_directories(${cv_bridge_INCLUDE_DIRS})

install(TARGETS
  lab1
  lab2
  lab3
  lab4
  scan_to_image_node_students
  lab5
  lab7

  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
