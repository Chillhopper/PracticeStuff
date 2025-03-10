cmake_minimum_required(VERSION 3.8)
project(image_action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()




# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED
  PATHS /opt/ros/humble/share/cv_bridge/cmake
  NO_DEFAULT_PATH)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the action interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ImageProcess.action"
  DEPENDENCIES sensor_msgs
)

# Export dependencies for runtime
ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Action Server
add_executable(image_action_server src/image_action_server.cpp)
ament_target_dependencies(image_action_server rclcpp rclcpp_action sensor_msgs cv_bridge image_transport rosidl_default_runtime)
rosidl_target_interfaces(image_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Action Client
add_executable(image_action_client src/image_action_client.cpp)
ament_target_dependencies(image_action_client rclcpp rclcpp_action sensor_msgs cv_bridge image_transport rosidl_default_runtime)
rosidl_target_interfaces(image_action_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executables
install(TARGETS
  image_action_server
  image_action_client
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
