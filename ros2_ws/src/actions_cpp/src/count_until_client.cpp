#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "astra_auto_interfaces/action/count_until.hpp"

using CountUntil = astra_auto_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilClientNode : public rclcpp::Node 
{
public:
    CountUntilClientNode() : Node("count_until_client") 
    {
        count_until_client_ = 
            rclcpp_action::create_client<CountUntil>(this, "count_until");
    }

    void send_goal(int target_number, double period)
    {
        //Wait for the Action Server
        count_until_client_->wait_for_action_server();

        // Create a goal
        auto goal = CountUntil::Goal();
        goal.target_number = target_number;
        goal.period = period;

        // Add callbacks
        auto options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        options.result_callback = 
            std::bind(&CountUntilClientNode::goal_result_callback, this, _1);

        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending a goal");
        count_until_client_->async_send_goal(goal, options);
    }
private:

    // Callback to receive the results once the goal is done
    void goal_result_callback(const CountUntilGoalHandle::WrappedResult &result)
    {
        int reached_number = result.result->reached_number;
        RCLCPP_INFO(this->get_logger(), "Result: %d", reached_number);
    }

    rclcpp_action::Client<CountUntil>::SharedPtr count_until_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClientNode>(); 
    node->send_goal(6, 0.8);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}