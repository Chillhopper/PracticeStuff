// simple_service_cpp/src/simple_service_client.cpp

#include "rclcpp/rclcpp.hpp"
#include "simple_service_cpp/srv/add_two_ints.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("simple_service_client");

  // Create a client for the "add_two_ints" service
  auto client = node->create_client<simple_service_cpp::srv::AddTwoInts>("add_two_ints");

  // Wait for the service to be available
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service");
      return 1;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service to appear...");
  }

  // Create a request and populate it
  auto request = std::make_shared<simple_service_cpp::srv::AddTwoInts::Request>();
  request->a = 5;
  request->b = 7;

  // Call the service and wait for the response
  auto result = client->async_send_request(request);

  // Wait for the response asynchronously
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result of add_two_ints: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call failed");
  }

  rclcpp::shutdown();
  return 0;
}

