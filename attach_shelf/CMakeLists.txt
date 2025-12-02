cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
  DEPENDENCIES std_msgs
)

# Pre-approach node (Task 1)
add_executable(pre_approach_node src/pre_approach.cpp)
ament_target_dependencies(pre_approach_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# Pre-approach v2 node (Task 2)
add_executable(pre_approach_v2_node src/pre_approach_v2.cpp)
ament_target_dependencies(pre_approach_v2_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)
rosidl_target_interfaces(pre_approach_v2_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Approach service server node (Task 2)
add_executable(approach_service_server_node src/approach_service_server.cpp)
ament_target_dependencies(approach_service_server_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
rosidl_target_interfaces(approach_service_server_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executables
install(TARGETS
  pre_approach_node
  pre_approach_v2_node
  approach_service_server_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}/
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
