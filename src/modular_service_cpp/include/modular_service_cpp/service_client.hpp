#ifndef MODULAR_SERVICE_CPP_SERVICE_CLIENT_HPP_
#define MODULAR_SERVICE_CPP_SERVICE_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "modular_service_cpp/srv/add_two_ints.hpp"

class ServiceClient : public rclcpp::Node
{
public:
    ServiceClient();

    void send_request(int64_t a, int64_t b);

private:
    rclcpp::Client<modular_service_cpp::srv::AddTwoInts>::SharedPtr client_;
};

#endif // MODULAR_SERVICE_CPP_SERVICE_CLIENT_HPP_

