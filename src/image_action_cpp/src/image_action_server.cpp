#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "image_action_cpp/action/image_process.hpp"

using namespace std::chrono_literals;
using ImageProcess = image_action_cpp::action::ImageProcess;
using GoalHandleImageProcess = rclcpp_action::ServerGoalHandle<ImageProcess>;

class ImageActionServer : public rclcpp::Node {
public:
    ImageActionServer() : Node("image_action_server") {
        action_server_ = rclcpp_action::create_server<ImageProcess>(
            this,
            "process_image",
            std::bind(&ImageActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ImageActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ImageActionServer::handle_accepted, this, std::placeholders::_1)
        );
    }

private:
    rclcpp_action::Server<ImageProcess>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ImageProcess::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received image for processing");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleImageProcess> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Cancel request received");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleImageProcess> goal_handle) {
        std::thread{std::bind(&ImageActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleImageProcess> goal_handle) {
        auto feedback = std::make_shared<ImageProcess::Feedback>();
        auto result = std::make_shared<ImageProcess::Result>();

        for (int i = 0; i <= 100; i += 20) {
            feedback->progress = "Processing... " + std::to_string(i) + "%";
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(1s);
        }

        result->success = true;
        result->message = "Image processing completed!";
        goal_handle->succeed(result);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageActionServer>());
    rclcpp::shutdown();
    return 0;
}

