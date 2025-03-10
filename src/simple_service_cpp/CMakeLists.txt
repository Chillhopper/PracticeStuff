cmake_minimum_required(VERSION 3.5)
project(simple_service_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare the service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
)


# Include directories (this ensures it includes generated files too)
include_directories(include)

# Ensure dependencies are correctly exported
ament_export_dependencies(rosidl_default_runtime)

# Ensure messages are available before building
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add executable for service server
add_executable(simple_service_server src/simple_service_server.cpp)
ament_target_dependencies(simple_service_server
  rclcpp
)
target_link_libraries(simple_service_server ${cpp_typesupport_target})

# Add executable for service client
add_executable(simple_service_client src/simple_service_client.cpp)
ament_target_dependencies(simple_service_client
  rclcpp
)
target_link_libraries(simple_service_client ${cpp_typesupport_target})

# Install targets
install(TARGETS
  simple_service_server
  simple_service_client
  DESTINATION lib/${PROJECT_NAME}
)

# Ensure generated messages are available before building
ament_export_dependencies(rosidl_default_runtime)

ament_package()

