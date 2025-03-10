// simple_service_cpp/src/simple_service_server.cpp

#include "rclcpp/rclcpp.hpp"
#include "simple_service_cpp/srv/add_two_ints.hpp"

void handle_service(
  const std::shared_ptr<simple_service_cpp::srv::AddTwoInts::Request> request,
  std::shared_ptr<simple_service_cpp::srv::AddTwoInts::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request: a=%ld, b=%ld, sum=%ld",
              request->a, request->b, response->sum);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("simple_service_server");

  auto service = node->create_service<simple_service_cpp::srv::AddTwoInts>(
    "add_two_ints", &handle_service);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service ready to add two integers");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

