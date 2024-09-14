//*************************************************************************************************
//rover-Autonomy Server
//State machine for ASTRA rover
//Last edited Sept 9, 2024
//*************************************************************************************************
//Maintained by: Daegan Brown
//Number: 423-475-4384
//Email: daeganbrown03@gmail.com
//*************************************************************************************************
//INCLUDES
//*************************************************************************************************

#include "rclcpp/rclcpp.hpp"                    //Needed for ros2 cpp 
#include "rclcpp_action/rclcpp_action.hpp"      //Needed for actions, specifically
#include "clucky_interfaces/action/auto_command.hpp"     
#include <string>                               //Gosh I love strings in c++


using AutoCommand = clucky_interfaces::action::AutoCommand;
using AutoCommandGoalHandle = rclcpp_action::ServerGoalHandle<AutoCommand>;
using namespace std::placeholders;

//*************************************************************************************************
// Global Variables
//*************************************************************************************************

int mission_type;                                               //State
double gps_lat_target, gps_long_target, target_radius, period;  //Inputs
double gps_lat_current, gps_long_current;                       //Current Location
std::string gps_string;                                         //Most recent GPS String

//*************************************************************************************************
// Server Nodes 
//*************************************************************************************************

// Subscriber Server node
class AutoServerSubscriberNode : public rclcpp::Node
{
    
};

// Primary Server Node
class AutoServerNode : public rclcpp::Node
{
    public:
        AutoServerNode() : Node("astra_auto_server")
        {
            astra_auto_server_ = rclcpp_action::create_server<AutoCommand>(
                this,
                "auto_command",
                std::bind(&AutoServerNode::goal_callback, this, _1, _2),
                std::bind(&AutoServerNode::cancel_callback, this, _1),
                std::bind(&AutoServerNode::handle_accepted_callback, this, _1)

                
            );
            RCLCPP_INFO(this->get_logger(), "Action server has been started uWu");
        }

    private:

        // Response Behavior
        rclcpp_action::GoalResponse goal_callback(
            const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const AutoCommand::Goal> goal)
            {
                // Get rid of STDR outputs
                (void)uuid;
                (void)goal;
                // set temp variable = gps targets, then make sure distance is < 1000 meters

                
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                // return rclcpp_action::GoalResponse::REJECT;
            }

        // Cancel Behavior
        rclcpp_action::CancelResponse cancel_callback(
            const std::shared_ptr<AutoCommandGoalHandle> goal_handle)
            {
                // Get rid of STDR outputs
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            }

        // Seems redundant
        void handle_accepted_callback(
            const std::shared_ptr<AutoCommandGoalHandle> goal_handle) 
            {
                // Pull request data from .action into global variables
                mission_type = goal_handle->get_goal()->mission_type;
                gps_lat_target = goal_handle->get_goal()->gps_lat_target;
                gps_long_target = goal_handle->get_goal()->gps_long_target;
                target_radius = goal_handle->get_goal()->target_radius;
                period = goal_handle->get_goal()->period;

                RCLCPP_INFO(this->get_logger(), "Executing the goal");
                execute_goal(goal_handle);
            }

        // Actual Code that runs when doing action request
        void execute_goal(
            const std::shared_ptr<AutoCommandGoalHandle> goal_handle)
            {
                
                // I dont remember why this is needed
                rclcpp::Rate loop_rate(2.0/period);

                //*********************************************************************************
                //                               FAKE PLACEHOLDER
                //*********************************************************************************
                int counter = 0;
                
                for (int i = 0; i < mission_type; i++)
                {
                    counter++;
                    RCLCPP_INFO(this->get_logger(), "%d", counter);
                    loop_rate.sleep();
                }
                //*********************************************************************************
                //                              END FAKE PLACEHOLDER
                //*********************************************************************************


                // Set final state and return result
                auto result = std::make_shared<AutoCommand::Result>();
                result->final_result = counter;
                goal_handle->succeed(result);
            }

        rclcpp_action::Server<AutoCommand>::SharedPtr astra_auto_server_;
        };



//*************************************************************************************************
// Main
//*************************************************************************************************

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}