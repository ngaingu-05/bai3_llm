cmake_minimum_required(VERSION 3.16)
project(ur3_llm_robot_skills)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(robot_skills src/robot_skills.cpp)
target_compile_features(robot_skills PUBLIC cxx_std_17)
target_link_libraries(robot_skills nlohmann_json::nlohmann_json)
ament_target_dependencies(robot_skills
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  shape_msgs
  std_msgs
  trajectory_msgs
)

install(TARGETS robot_skills DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config urdf worlds DESTINATION share/${PROJECT_NAME})

ament_package()