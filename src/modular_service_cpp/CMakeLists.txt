cmake_minimum_required(VERSION 3.8)
project(modular_service_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the service interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
)

# Export runtime dependencies for generated messages
ament_export_dependencies(rosidl_default_runtime)

# Include directories for custom and generated headers
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Build the service server
add_executable(service_server src/service_server.cpp src/service_server_impl.cpp)
ament_target_dependencies(service_server rclcpp rosidl_default_runtime)
rosidl_target_interfaces(service_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Build the service client
add_executable(service_client src/service_client.cpp src/service_client_impl.cpp)
ament_target_dependencies(service_client rclcpp rosidl_default_runtime)
rosidl_target_interfaces(service_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executables
install(TARGETS
  service_server
  service_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

