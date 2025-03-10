#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "simple_action_cpp/action/count_until.hpp"

using namespace std::chrono_literals;
using CountUntil = simple_action_cpp::action::CountUntil;
using GoalHandleCountUntil = rclcpp_action::ServerGoalHandle<CountUntil>;

class CountActionServer : public rclcpp::Node
{
public:
    CountActionServer() : Node("count_action_server")
    {
        this->action_server_ = rclcpp_action::create_server<CountUntil>(
            this,
            "count_until",
            std::bind(&CountActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CountActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&CountActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<CountUntil>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const CountUntil::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal to count until: %d", goal->goal_count);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
    {
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    void execute(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
    {
        auto feedback = std::make_shared<CountUntil::Feedback>();
        auto result = std::make_shared<CountUntil::Result>();

        for (int i = 1; i <= goal_handle->get_goal()->goal_count; ++i)
        {
            if (goal_handle->is_canceling()) {
                result->final_count = i;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal Canceled at %d", i);
                return;
            }

            feedback->current_number = i;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publishing feedback: %d", i);
            std::this_thread::sleep_for(1s);
        }

        if (rclcpp::ok()) {
            result->final_count = goal_handle->get_goal()->goal_count;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

