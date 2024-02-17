//***********************************************
//rover-Autonomy Server
//runs commands from the client
//Last edited Feb 17, 2024
//Version: 1.1
//***********************************************
//Maintained by: Daegan Brown
//Number: 423-475-4384
//Email: daeganbrown03@gmail.com
//***********************************************
#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <unistd.h>

#include "pathfind.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/subscription_options.hpp"

#include "std_msgs/msg/string.hpp"

#include "astra_auto_interfaces/action/navigate_rover.hpp"



using NavigateRover = astra_auto_interfaces::action::NavigateRover;
using NavigateRoverGoalHandle = rclcpp_action::ServerGoalHandle<NavigateRover>;
using namespace std::placeholders;


std::string imu_bearing;

class NavigateRoverServerNode : public rclcpp::Node 
{
public:
    NavigateRoverServerNode() : Node("navigate_rover_server"), count_(0) 
    {
        //Creating action
        navigate_rover_server_ = rclcpp_action::create_server<NavigateRover>(
                this,
                "navigate_rover",
                std::bind(&NavigateRoverServerNode::goal_callback, this, _1, _2),
                std::bind(&NavigateRoverServerNode::cancel_callback, this, _1),
                std::bind(&NavigateRoverServerNode::handle_accepted_callback, this, _1)
            );
        RCLCPP_INFO(this->get_logger(), "Action server has been started");
        
        //Creating Publisher that communicates to the motors
        publisher_motors = this->create_publisher<std_msgs::msg::String>(
            "astra/core/control", 10);
        
        //Creating a publisher that asks for imu data
        publisher_imu = this->create_publisher<std_msgs::msg::String>(
            "astra/core/control", 10);
        

        //Creating a Subscriber that listens to astra/core/feedback for IMU Heading
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "astra/core/feedback", 10, std::bind(&NavigateRoverServerNode::topic_callback_imu, this, _1));
        
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
        int navigate_type = goal_handle->get_goal()->navigate_type;
        double gps_lat_target = goal_handle->get_goal()->gps_lat_target;
        double gps_long_target = goal_handle->get_goal()->gps_long_target;
        double period = goal_handle->get_goal()->period;

        // Execute the action
        int final_result = 0;
        std::string rover_command;
        rclcpp::Rate loop_rate(1.0/period);
        
        // Switch Statement determining what type of action is being asked of the 
        // rover. 
        // 1: Simply go to GPS coordinates, stop, and signal.
        // 2: Go and search target area for aruco tags
        // 3: Go and search target area for objects
        // 4: Stops rover where it is
        // 5: Goes forward. Used for testing. 
        auto message_motors = std_msgs::msg::String();
        auto message_imu = std_msgs::msg::String();
        double bearing;
        int iterate = 0;

        if (navigate_type == 1)
        {
            while (iterate == 0)
            {
                message_imu.data = "auto,rotateTo,15,0";
                publisher_imu->publish(message_imu);
                
                pathfindFunctions locate_point;
                pathfindFunctions gps_data;
                message_imu.data = "data,getOrientation";
                publisher_imu->publish(message_imu);
                usleep(100000);
                float currentHeading = std::stof(imu_bearing);

                message_imu.data = "data,getGPS";
                publisher_imu->publish(message_imu);
                usleep(100000);
                double current_lat = gps_data.imu_command_gps(imu_bearing,1);
                double current_long = gps_data.imu_command_gps(imu_bearing,2);


                float needHeading = locate_point.find_facing(gps_lat_target, gps_long_target, currentHeading, current_lat, current_long);
                int i_needHeading = needHeading;

                message_imu.data = "auto,rotateTo,15," + i_needHeading;
                publisher_imu->publish(message_imu);


                rover_command = "ctrl,.20,.20";  
                message_motors.data = rover_command;
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_motors.data.c_str());
                publisher_motors->publish(message_motors);
                usleep(3 * microsecond);

                rover_command = "ctrl,0,0";  
                message_motors.data = rover_command;
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_motors.data.c_str());
                publisher_motors->publish(message_motors);

                
                iterate++;
            }
             
            
            RCLCPP_INFO(this->get_logger(), "At location");            
            
        }
        else if (navigate_type == 5)
        {
            rover_command = "ctrl,.50,.50";
                
            
            message_motors.data = rover_command;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_motors.data.c_str());
            publisher_motors->publish(message_motors);

            // Do a little spin
            usleep(3 * microsecond);
            message_motors.data = "ctrl,-0.5,.5";
            RCLCPP_INFO(this->get_logger(), "Turning");
        }
        
        

        // Pause before stopping 

        usleep(3 * microsecond);
        message_motors.data = "ctrl,0,0";
        RCLCPP_INFO(this->get_logger(), "Stopping");
        publisher_motors->publish(message_motors);

        
        


        // Set final state and return result
        auto result = std::make_shared<NavigateRover::Result>();
        result->final_result = final_result;
        goal_handle->succeed(result);
    }

    //Subscriber to astra/core/feedback
    
    void topic_callback_imu(const std_msgs::msg::String & msg) 
    {
        std::string command;
        RCLCPP_INFO(this->get_logger(), "Recieved IMU: '%s'", msg.data.c_str());
        command = msg.data;



        pathfindFunctions findBearing;
        imu_bearing = findBearing.imu_command(command);

    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_motors;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_imu;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
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