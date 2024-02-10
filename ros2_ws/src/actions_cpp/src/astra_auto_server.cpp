#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/subscription_options.hpp"

#include "std_msgs/msg/string.hpp"

#include "astra_auto_interfaces/action/navigate_rover.hpp"



using NavigateRover = astra_auto_interfaces::action::NavigateRover;
using NavigateRoverGoalHandle = rclcpp_action::ServerGoalHandle<NavigateRover>;
using namespace std::placeholders;


class NavigateRoverServerNode : public rclcpp::Node 
{
public:
    NavigateRoverServerNode() : Node("navigate_rover_server"), count_(0) 
    {
        navigate_rover_server_ = rclcpp_action::create_server<NavigateRover>(
                this,
                "navigate_rover",
                std::bind(&NavigateRoverServerNode::goal_callback, this, _1, _2),
                std::bind(&NavigateRoverServerNode::cancel_callback, this, _1),
                std::bind(&NavigateRoverServerNode::handle_accepted_callback, this, _1)
            );
            RCLCPP_INFO(this->get_logger(), "Action server has been started");

        publisher_ = this->create_publisher<std_msgs::msg::String>("astra/core/control", 10);
        /*timer_ = this->create_wall_timer(
            500ms, std::bind(&navigate_rover_server::timer_callback,this));*/
        
    }

private:

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const NavigateRover::Goal> goal)
    {
        //to get rid of startup warnings
        (void)uuid;
        (void)goal;
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<NavigateRoverGoalHandle> goal_handle)
    {
        //to get rid of startup warnings
        (void)goal_handle;

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(
        const std::shared_ptr<NavigateRoverGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing the goal");
        execute_goal(goal_handle);
    }

    void execute_goal(
        const std::shared_ptr<NavigateRoverGoalHandle> goal_handle)
    {
        // Set microsecond values for usleep command
        unsigned int microsecond = 1000000;

        // Get Request from goal
        int test_number = goal_handle->get_goal()->test_number;
        double test_float = goal_handle->get_goal()->test_float;
        double period = goal_handle->get_goal()->period;

        // Execute the action
        int final_result = 0;
        std::string rover_command;
        rclcpp::Rate loop_rate(1.0/period);
        
        // Switch Statement determining what type of action is being asked of the 
        // rover. 
        // targetPoint: Simply go to GPS coordinates, stop, and signal.
        // targetAruco: Go and search target area for aruco tags
        // targetObject: Go and search target area for objects
        // ABORT: Stops rover where it is
        // testShimmey: Goes forward. Used for testing. 
        switch (test_number)
        {
            case 5:
                rover_command = "ctrl, .50, .50";
                break;
            default:
                final_result = 1;
        }
        auto message = std_msgs::msg::String();
        message.data = rover_command;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);

        // Pause before stopping 

        usleep(3 * microsecond);
        message.data = "ctrl, 0, 0";
        RCLCPP_INFO(this->get_logger(), "Stopping");
        publisher_->publish(message);

        
        


        // Set final state and return result
        auto result = std::make_shared<NavigateRover::Result>();
        result->final_result = final_result;
        goal_handle->succeed(result);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    rclcpp_action::Server<NavigateRover>::SharedPtr navigate_rover_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateRoverServerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}