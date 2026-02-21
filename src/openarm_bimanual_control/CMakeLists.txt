cmake_minimum_required(VERSION 3.8)
project(openarm_bimanual_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# Trajectory merger library (no ROS in implementation, only msgs)
add_library(trajectory_merger_lib src/trajectory_merger.cpp)
target_include_directories(trajectory_merger_lib PUBLIC include)
ament_target_dependencies(trajectory_merger_lib trajectory_msgs builtin_interfaces)

# Merge-execute node
add_executable(merge_execute_node src/merge_execute_node.cpp)
target_include_directories(merge_execute_node PRIVATE include)
ament_target_dependencies(merge_execute_node
  rclcpp
  rclcpp_action
  trajectory_msgs
  control_msgs
  moveit_ros_planning_interface
  moveit_msgs
)
target_link_libraries(merge_execute_node trajectory_merger_lib)

# Install
install(TARGETS
  trajectory_merger_lib
  merge_execute_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Export
ament_export_include_directories(include)
ament_export_libraries(trajectory_merger_lib)
ament_export_dependencies(
  trajectory_msgs
  control_msgs
  builtin_interfaces
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
