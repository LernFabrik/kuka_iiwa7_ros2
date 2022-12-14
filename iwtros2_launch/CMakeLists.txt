cmake_minimum_required(VERSION 3.8)
project(iwtros2_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(iwtros2_interface REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/iiwa_manipulation.cpp
  src/create_motion_planning.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  control_msgs
  geometry_msgs
  tf2_ros
  moveit
  moveit_visual_tools
  moveit_ros_planning_interface
  iwtros2_interface
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

add_executable(iiwa7_manipulation_node src/main.cpp)
target_link_libraries(iiwa7_manipulation_node ${PROJECT_NAME})
target_include_directories(iiwa7_manipulation_node PRIVATE include)
ament_target_dependencies(iiwa7_manipulation_node
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  control_msgs
  geometry_msgs
  tf2_ros
  moveit
  moveit_visual_tools
  moveit_ros_planning_interface
  iwtros2_interface
)

add_executable(iiwa7_manipulation_test_node src/test.cpp)
target_link_libraries(iiwa7_manipulation_test_node ${PROJECT_NAME})
target_include_directories(iiwa7_manipulation_test_node PRIVATE include)
ament_target_dependencies(iiwa7_manipulation_test_node
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  control_msgs
  geometry_msgs
  tf2_ros
  moveit
  moveit_visual_tools
  moveit_ros_planning_interface
  iwtros2_interface
)

add_executable(planning_scene_monitor_node src/planning_scene_monitor.cpp)
target_link_libraries(planning_scene_monitor_node ${PROJECT_NAME})
target_include_directories(planning_scene_monitor_node PRIVATE include)
ament_target_dependencies(planning_scene_monitor_node
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  control_msgs
  geometry_msgs
  tf2_ros
  moveit
  moveit_visual_tools
  moveit_ros_planning_interface
  iwtros2_interface
)

add_executable(joint_state_combine_node src/joint_state_combine.cpp)
target_link_libraries(joint_state_combine_node ${PROJECT_NAME})
target_include_directories(joint_state_combine_node PRIVATE include)
ament_target_dependencies(joint_state_combine_node
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  control_msgs
  geometry_msgs
  tf2_ros
  moveit
  moveit_visual_tools
  moveit_ros_planning_interface
  iwtros2_interface
)
# rclcpp_components_register_node(iiwa7_manipulation_node PLUGIN "iwtros2::IiwaMove" EXECUTABLE iiwa_motion_controller_node)
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  iiwa7_manipulation_node
  iiwa7_manipulation_test_node
  planning_scene_monitor_node
  joint_state_combine_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/ros2_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
