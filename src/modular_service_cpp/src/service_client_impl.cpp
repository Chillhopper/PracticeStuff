#include "modular_service_cpp/service_client.hpp"

ServiceClient::ServiceClient() : Node("modular_service_client")
{
    client_ = this->create_client<modular_service_cpp::srv::AddTwoInts>("add_two_ints");
}

void ServiceClient::send_request(int64_t a, int64_t b)
{
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }

    auto request = std::make_shared<modular_service_cpp::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    auto result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Result: %ld", result.get()->sum);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    }
}

