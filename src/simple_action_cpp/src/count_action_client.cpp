#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "simple_action_cpp/action/count_until.hpp"

using namespace std::chrono_literals;
using CountUntil = simple_action_cpp::action::CountUntil;
using GoalHandleCountUntil = rclcpp_action::ClientGoalHandle<CountUntil>;

class CountActionClient : public rclcpp::Node
{
public:
    CountActionClient() : Node("count_action_client")
    {
        this->client_ptr_ = rclcpp_action::create_client<CountUntil>(this, "count_until");
    }

    void send_goal(int count)
    {
        while (!this->client_ptr_->wait_for_action_server(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        }

        auto goal_msg = CountUntil::Goal();
        goal_msg.goal_count = count;

        auto send_goal_options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        send_goal_options.feedback_callback = std::bind(&CountActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&CountActionClient::result_callback, this, std::placeholders::_1);

        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<CountUntil>::SharedPtr client_ptr_;

    void feedback_callback(GoalHandleCountUntil::SharedPtr, const std::shared_ptr<const CountUntil::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Current Count: %d", feedback->current_number);
    }

    void result_callback(const GoalHandleCountUntil::WrappedResult &result)
    {
        RCLCPP_INFO(this->get_logger(), "Final Count: %d", result.result->final_count);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountActionClient>();
    node->send_goal(5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

