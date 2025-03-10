#include "modular_service_cpp/service_server.hpp"

ServiceServer::ServiceServer() : Node("modular_service_server")
{
    service_ = this->create_service<modular_service_cpp::srv::AddTwoInts>(
        "add_two_ints",
        std::bind(&ServiceServer::handle_request, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Service Server Ready");
}

void ServiceServer::handle_request(
    const std::shared_ptr<modular_service_cpp::srv::AddTwoInts::Request> request,
    std::shared_ptr<modular_service_cpp::srv::AddTwoInts::Response> response)
{
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "Received request: %ld + %ld = %ld", request->a, request->b, response->sum);
}

