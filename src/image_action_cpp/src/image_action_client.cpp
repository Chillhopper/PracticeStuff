#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "image_action_cpp/action/image_process.hpp"

using ImageProcess = image_action_cpp::action::ImageProcess;

class ImageActionClient : public rclcpp::Node {
public:
    ImageActionClient() : Node("image_action_client") {
        client_ = rclcpp_action::create_client<ImageProcess>(this, "process_image");

        while (!client_->wait_for_action_server()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        }

        auto goal = ImageProcess::Goal();
        goal.image = sensor_msgs::msg::Image(); // Placeholder for image data

        auto send_goal_options = rclcpp_action::Client<ImageProcess>::SendGoalOptions();
        send_goal_options.feedback_callback = [](auto, const auto feedback) {
            RCLCPP_INFO(rclcpp::get_logger("client"), "Feedback: %s", feedback->progress.c_str());
        };

        client_->async_send_goal(goal, send_goal_options);
    }

private:
    rclcpp_action::Client<ImageProcess>::SharedPtr client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageActionClient>());
    rclcpp::shutdown();
    return 0;
}

