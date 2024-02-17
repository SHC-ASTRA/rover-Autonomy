//***********************************************
//rover-Autonomy server client
//Sends instructions to the server
//Last edited Feb 16, 2024
//Version: 1.1
//***********************************************
//Maintained by: Daegan Brown
//Number: 423-475-4384
//Email: daeganbrown03@gmail.com
//***********************************************
#include <iostream>

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

    void send_goal(int navigate_type, double gps_lat_target, 
        double gps_long_target, double period)
    {
        //Wait for the Action Server
        navigate_rover_client_->wait_for_action_server();

        // Create a goal
        auto goal = NavigateRover::Goal();
        goal.navigate_type = navigate_type;
        goal.gps_lat_target = gps_lat_target;
        goal.gps_long_target = gps_long_target;
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
    int navigate_type;
    double gps_lat_target;
    double gps_long_target;

    //Which type of navigation. See astra_auto_server.cpp for a list of
    //options. 
    std::cout << "Input Type:" << std::endl;
    std::cin >> navigate_type; 
    std::cout << std::endl; 

    //The target latitude co-ordinate.
    //8 decimal places
    std::cout << "Target Latitude:" << std::endl;
    std::cin >> gps_lat_target; 
    std::cout << std::endl; 

    //The target longitude co-ordinate.
    //8 decimal places
    std::cout << "Target Longitude:" << std::endl;
    std::cin >> gps_long_target; 
    std::cout << std::endl; 


    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateRoverClientNode>(); 
    node->send_goal(navigate_type, gps_lat_target, gps_long_target, 0.8);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}