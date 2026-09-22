cmake_minimum_required(VERSION 3.8)
project(ur3_writer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(ur3_writer_node src/ur3_writer_node.cpp)
target_include_directories(ur3_writer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ur3_writer_node PUBLIC c_std_17)

ament_target_dependencies(ur3_writer_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  visualization_msgs
)

install(TARGETS ur3_writer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
