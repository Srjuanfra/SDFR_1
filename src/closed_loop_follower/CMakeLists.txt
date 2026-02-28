cmake_minimum_required(VERSION 3.8)
project(closed_loop_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(closed_loop_follower_node src/closed_loop_follower_node.cpp)
ament_target_dependencies(closed_loop_follower_node
  rclcpp
  example_interfaces
  geometry_msgs
  std_msgs
)

install(TARGETS closed_loop_follower_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
