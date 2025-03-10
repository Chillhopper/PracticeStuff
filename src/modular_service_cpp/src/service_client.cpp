#include "modular_service_cpp/service_client.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceClient>();
    node->send_request(10, 20);
    rclcpp::shutdown();
    return 0;
}

