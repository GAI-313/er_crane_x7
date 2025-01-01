cmake_minimum_required(VERSION 3.8)
project(er_crane_x7_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(er_crane_x7_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(sensor_msgs REQUIRED)  # sensor_msgsの追加

# Add executable for move_groupstate_server
add_executable(move_groupstate_server src/move_groupstate_server.cpp)
ament_target_dependencies(move_groupstate_server 
  rclcpp 
  moveit_ros_planning_interface 
  er_crane_x7_srvs
  geometry_msgs
  tf2_geometry_msgs
  angles
)

# Add executable for robot_control_server
add_executable(arm_pose_server src/arm_pose_server.cpp)
ament_target_dependencies(arm_pose_server 
  rclcpp 
  moveit_ros_planning_interface 
  er_crane_x7_srvs
  geometry_msgs
  tf2_geometry_msgs
  angles
)

# Add executable for gripper_server
add_executable(gripper_server src/gripper_server.cpp)
ament_target_dependencies(gripper_server 
  rclcpp 
  moveit_ros_planning_interface 
  er_crane_x7_srvs
  geometry_msgs
  tf2_geometry_msgs
  angles
)

# Install targets
install(TARGETS
  move_groupstate_server
  arm_pose_server
  gripper_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install other necessary directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Setup for testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
