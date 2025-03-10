#include "modular_service_cpp/service_server.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

