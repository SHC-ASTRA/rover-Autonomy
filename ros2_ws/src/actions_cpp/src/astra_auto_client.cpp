#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "astra_auto_interfaces/action/navigate_rover.hpp"

using NavigateRover = astra_auto_interfaces::action::NavigateRover;
using NavigateRoverGoalHandle = rclcpp_action::ClientGoalHandle<NavigateRover>;
using namespace std::placeholders;

class NavigateRoverClientNode : public rclcpp::Node 
{
public:
    NavigateRoverClientNode() : Node("navigate_rover_client") 
    {
        navigate_rover_client_ = 
            rclcpp_action::create_client<NavigateRover>(this, "navigate_rover");
    }

    void send_goal(int test_number, double test_float, double period)
    {
        //Wait for the Action Server
        navigate_rover_client_->wait_for_action_server();

        // Create a goal
        auto goal = NavigateRover::Goal();
        goal.test_number = test_number;
        goal.test_float = test_float;
        goal.period = period;

        // Add callbacks
        auto options = rclcpp_action::Client<NavigateRover>::SendGoalOptions();
        options.result_callback = 
            std::bind(&NavigateRoverClientNode::goal_result_callback, this, _1);

        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending a goal");
        navigate_rover_client_->async_send_goal(goal, options);
    }
private:

    // Callback to receive the results once the goal is done
    void goal_result_callback(const NavigateRoverGoalHandle::WrappedResult &result)
    {
        int final_result = result.result->final_result;
        RCLCPP_INFO(this->get_logger(), "Result: %d", final_result);
    }

    rclcpp_action::Client<NavigateRover>::SharedPtr navigate_rover_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateRoverClientNode>(); 
    node->send_goal(5, 6, 0.8);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}