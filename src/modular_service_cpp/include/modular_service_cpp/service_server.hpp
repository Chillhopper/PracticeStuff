#ifndef MODULAR_SERVICE_CPP_SERVICE_SERVER_HPP_
#define MODULAR_SERVICE_CPP_SERVICE_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "modular_service_cpp/srv/add_two_ints.hpp"

class ServiceServer : public rclcpp::Node
{
public:
    ServiceServer();

private:
    rclcpp::Service<modular_service_cpp::srv::AddTwoInts>::SharedPtr service_;

    void handle_request(
        const std::shared_ptr<modular_service_cpp::srv::AddTwoInts::Request> request,
        std::shared_ptr<modular_service_cpp::srv::AddTwoInts::Response> response);
};

#endif // MODULAR_SERVICE_CPP_SERVICE_SERVER_HPP_

