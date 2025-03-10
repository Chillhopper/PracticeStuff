cmake_minimum_required(VERSION 3.8)
project(simple_action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountUntil.action"
)

add_executable(action_server src/count_action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action)
rosidl_target_interfaces(action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Action Client
add_executable(action_client src/count_action_client.cpp)
ament_target_dependencies(action_client rclcpp rclcpp_action)
rosidl_target_interfaces(action_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

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

install(TARGETS
  action_server
  action_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
