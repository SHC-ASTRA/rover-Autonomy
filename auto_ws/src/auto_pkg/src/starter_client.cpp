//*************************************************************************************************
//rover-Autonomy Startup Client
//State machine for ASTRA rover
//Last edited Sept 9, 2024
//*************************************************************************************************
//Maintained by: Daegan Brown
//Number: 423-475-4384
//Email: daeganbrown03@gmail.com
//*************************************************************************************************
//INCLUDES
//*************************************************************************************************

#include "rclcpp/rclcpp.hpp"                            //Needed for all cpp ROS files
#include "rclcpp_action/rclcpp_action.hpp"              //Needed for all ROS actions
#include "clucky_interfaces/action/auto_command.hpp"    //My interface

using AutoCommand = clucky_interfaces::action::AutoCommand;
using AutoCommandGoalHandle = rclcpp_action:: ClientGoalHandle<AutoCommand>;
using namespace std::placeholders;
//*************************************************************************************************
// Client Node
//*************************************************************************************************

class AutoStartupClientNode : public rclcpp::Node
{
    public:
        AutoStartupClientNode() : Node("auto_client_start")
        {
            auto_client_start_ = 
                rclcpp_action::create_client<AutoCommand>(this, "auto_command");
        }

        void send_goal(int mission_type, double gps_lat_target, double gps_long_target, 
            double target_radius, double period)
        {
            // Wait for action Server
            auto_client_start_->wait_for_action_server();

            // Get instructions 
            // std::cout << "Enter Mission Type: ";
            // std::cin >> mission_type;
            // std::cout << std::endl;

            // std::cout << "Enter GPS Target Latitude: ";
            // std::cin >> gps_lat_target;
            // std::cout << std::endl;

            // std::cout << "Enter GPS Target Longitude: ";
            // std::cin >> gps_long_target;
            // std::cout << std::endl;

            // std::cout << "Enter Target Radius: ";
            // std::cin >> mission_type;
            // std::cout << std::endl;


            // Create a goal
            
            auto goal = AutoCommand::Goal();
            goal.mission_type = mission_type;
            goal.gps_lat_target = gps_lat_target;
            goal.gps_long_target = gps_long_target;
            goal.target_radius = target_radius;
            goal.period = period;
            
            //Add callbacks
            auto options = rclcpp_action::Client<AutoCommand>::SendGoalOptions();
            options.result_callback = 
                std::bind(&AutoStartupClientNode::goal_result_callback, this, _1);
            options.goal_response_callback =
                std::bind(&AutoStartupClientNode::goal_response_callback, this, _1);


            // Send it
            RCLCPP_INFO(this->get_logger(), "Sending a goal");
            auto_client_start_->async_send_goal(goal, options);

            // Wait for callback
        }
    private:
        //Callback to know if the goal was accepted or rejected
        void goal_response_callback(const AutoCommandGoalHandle::SharedPtr &goal_handle)
        {
            if(!goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Goal got rejected");
            }
            else 
            {
                RCLCPP_INFO(this->get_logger(), "Goal got accepted");
            }
        }

        //Callback for resukt
        void goal_result_callback(const AutoCommandGoalHandle::WrappedResult &result)
        {
            int exit_code = result.result->final_result;
            RCLCPP_INFO(this->get_logger(), "Result: %d", exit_code);
        }
    rclcpp_action::Client<AutoCommand>::SharedPtr auto_client_start_;
};

//*************************************************************************************************
// Main
//*************************************************************************************************

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoStartupClientNode>();
    node->send_goal(10, .8, .8, .8, .8);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
