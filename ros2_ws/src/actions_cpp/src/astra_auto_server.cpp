//***********************************************
//rover-Autonomy Server
//runs commands from the client
//Last edited March 30, 2024
//Version: 1.5
//*************************************************************************************************
//Maintained by: Daegan Brown
//Number: 423-475-4384
//Email: daeganbrown03@gmail.com
//*************************************************************************************************
//INCLUDES
//*************************************************************************************************

//C++ includes, normal
#include <memory>                           // 
#include <chrono>                           // 
#include <functional>                       // 
#include <string>                           // String type variable
#include <unistd.h>                         // usleep 
#include <stdio.h>

//Made by Daegan Brown for ASTRA
#include "pathfind.h"                       // My functions

//ROS2 includes
#include "rclcpp/rclcpp.hpp"                // General ROS2 stuff
#include "rclcpp_action/rclcpp_action.hpp"  // ROS2 actions info
#include "rclcpp/subscription_options.hpp"  // ROS2 subsriber info
#include "std_msgs/msg/string.hpp"          // Message type for ROS2
#include "image_transport/image_transport.hpp" // Image Transport for ROS


// Open CV ROS Bridge
#include "cv_bridge/cv_bridge.h"

//openCV shenanigans
#include <opencv2/opencv.hpp>                   //
#include <opencv2/core.hpp>                     //
#include <opencv2/aruco.hpp>                    //
#include <opencv2/videoio.hpp>                  //
#include <opencv2/highgui.hpp>                  //
#include <opencv2/objdetect/aruco_detector.hpp> //
#include <opencv2/calib3d.hpp>                  //


//Other packages to include
#include "astra_auto_interfaces/action/navigate_rover.hpp"  //contains action files and srv files


//*************************************************************************************************
//Predeclarations
//*************************************************************************************************


//Shorthands and other such things
using NavigateRover = astra_auto_interfaces::action::NavigateRover;
using NavigateRoverGoalHandle = rclcpp_action::ServerGoalHandle<NavigateRover>;
using namespace std::placeholders;


//Global Variables
std::string imu_bearing;                    
std::string gps_string;


//*************************************************************************************************
//ROS2 Nodes
//*************************************************************************************************


// Node for subcribing to topics
class NavigateRoverSubscriberNode : public rclcpp::Node 
{
public:
    
    NavigateRoverSubscriberNode() : Node("navigate_rover_subscriber")
    {
      navigate_rover_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "astra/core/feedback", 10, std::bind(&NavigateRoverSubscriberNode::topic_callback, this, _1));

      
    }

private:
    void topic_callback(const std_msgs::msg::String & msg) 
    {
        std::string command;
        command = msg.data;
        RCLCPP_INFO(this->get_logger(), "Recieved: '%s'", msg.data.c_str());
        

        
        std::string delimiter = ",";
            size_t pos = 0;
            std::string token;
            std::string scommand = command.c_str();
            pos = scommand.find(delimiter);
            token = scommand.substr(0, pos);
            

        if (token == "orientation")
        {
            RCLCPP_INFO(this->get_logger(), "Recieved IMU bearing");
            //Turns command into the proper bearing
            //imu_bearing = imu_command(command); 
        }
        else if (token == "gps")
        {
            RCLCPP_INFO(this->get_logger(), "Recieved GPS location");
            //Turns command into GPS string
            gps_string = command;
        }

    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigate_rover_subscriber_;

};

// Node for the action server
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

        //Creating Image Publisher Node and Publisher
        image_transport::ImageTransport image_transport_node(this);
        image_transport::Publisher image_publisher = image_transport_node.advertise('camera/compressed');
        
        //Creating Publisher that communicates to the motors
        publisher_motors = this->create_publisher<std_msgs::msg::String>(
            "astra/core/control", 10);
        
        //Creating a publisher that asks for imu data
        publisher_feedback = this->create_publisher<std_msgs::msg::String>(
            "astra/auto/feedback", 10);
        
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

        (void)goal_handle;
        std::string rover_command;
        auto message_motors = std_msgs::msg::String();

        rover_command = "ctrl,0,0";  
        message_motors.data = rover_command;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_motors.data.c_str());
        publisher_motors->publish(message_motors);

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
        // DEBUG change variable names
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
        // 0: Stops rover
        // 1: Simply go to GPS coordinates, stop, and signal.
        // 2: Go and search target area for aruco tags
        // 3: Go and search target area for objects
        // 4: 1 but only looping once
        // 5: Goes forward. Used for testing. 
        // 6: Search pattern
        // 7: AruCo Test
        // 8: Object Detection
        auto message_motors = std_msgs::msg::String();
        auto message_feedback = std_msgs::msg::String();
        double current_lat;
        double current_long;
        //double bearing;
        //float currentHeading;
        //float needHeading = 0;
        double needDistance;
        int i_needDistance;
        int i_needHeading;
        int iterate = 0;


        //Turn LEDs blue 
        message_motors.data = "led_set,300,0,0";
        publisher_motors->publish(message_motors);

        //Request GPS data from Core, then wait 3 seconds.
        message_motors.data = "data,sendGPS";
        publisher_motors->publish(message_motors);
        usleep(3 * microsecond);

        //Use first input to decide where to go. Switch statement could work
        //better, TBD
        if (navigate_type == 1)
        {
            //Make Rover face North, announce chosen task,
            message_motors.data = "auto,turningTo,15000,0";
            publisher_motors->publish(message_motors);
            std::cout << "Selected GPS targeting" << std::endl;

            //This loop gets the rover to continually get closer to the GPS 
            //target until it is within a half meter. 
            while (iterate == 0)
            {
                
                
                message_motors.data = "data,getOrientation";
                publisher_motors->publish(message_motors);
                usleep(0.5 * microsecond);
                
                

                message_motors.data = "data,sendGPS";
                publisher_motors->publish(message_motors);
                usleep(0.5 * microsecond);
                
                current_lat = imu_command_gps(gps_string,1);
                current_long = imu_command_gps(gps_string,2);

                needDistance = find_distance(gps_lat_target, gps_long_target, current_lat, current_long);
                i_needDistance = needDistance;
                RCLCPP_INFO(this->get_logger(), "Remaining distance: '%d'", i_needDistance);


                i_needHeading = find_facing(gps_lat_target, gps_long_target, current_lat, current_long);
                
                
                std::cout << std::fixed << "Calculated Heading: " << i_needHeading << std::endl << std::endl << std::endl << std::endl;


                message_motors.data = "auto,turningTo,15000," + std::to_string(i_needHeading);
                publisher_motors->publish(message_motors);
                usleep(3.5 * microsecond);

                
                rover_command = "ctrl,-0.6,-0.6";  
                message_motors.data = rover_command;
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_motors.data.c_str());
                publisher_motors->publish(message_motors);
                usleep(1.5 * microsecond);
                

                rover_command = "ctrl,0,0";  
                message_motors.data = rover_command;
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_motors.data.c_str());
                publisher_motors->publish(message_motors);

                message_motors.data = "data,getGPS";
                publisher_motors->publish(message_motors);
                usleep(100000);
                current_lat = imu_command_gps(gps_string,1);
                current_long = imu_command_gps(gps_string,2);

                if ((abs(current_lat - gps_lat_target) <= 0.00002) && \
                    ((abs(current_long - gps_long_target) <= 0.00002) ))
                {
                    
                    iterate++;
                }
                
            }
             
            message_motors.data = "led_set,0,0,300";
            publisher_motors->publish(message_motors);
            RCLCPP_INFO(this->get_logger(), "Arrived at location"); 
            std::cout << "Arrived at location!";           
            
        }
        //This is a test loop. Same code as 1, the only difference is instead of drving until it reaches the point, 
        //It faces it once, drives towards it once, then is done. 
        else if (navigate_type == 4)
        {
            std::cout << "Selected GPS targeting, but once" << std::endl;
            while (iterate == 0)
            {
                message_motors.data = "auto,turningTo,15000,0";
                
                publisher_motors->publish(message_motors);
                //usleep(15 * microsecond);
                
                
                
                message_motors.data = "data,getOrientation";
                publisher_motors->publish(message_motors);
                usleep(3 * microsecond);
                for (int i; i<5000; i++)
                {
                    usleep(1000);
                }

                message_motors.data = "data,sendGPS";
                publisher_motors->publish(message_motors);
                //usleep(3 * microsecond);
                for (int i; i<5000; i++)
                {
                    usleep(1000);
                }
                std::cout << gps_string << std::endl;  
                current_lat = imu_command_gps(gps_string,1);
                current_long = imu_command_gps(gps_string,2);


                i_needHeading = find_facing(gps_lat_target, gps_long_target, current_lat, current_long);
                
                
                std::cout << std::fixed << "Calculated Heading: " << i_needHeading << std::endl << std::endl << std::endl << std::endl;


                message_motors.data = "auto,turningTo,15000," + std::to_string(i_needHeading);
                publisher_motors->publish(message_motors);
                usleep(15 * microsecond);

                rover_command = "ctrl,0,0";  
                message_motors.data = rover_command;
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_motors.data.c_str());
                publisher_motors->publish(message_motors);

                message_motors.data = "data,getGPS";
                publisher_motors->publish(message_motors);
                usleep(100000);
                current_lat = imu_command_gps(gps_string,1);
                current_long = imu_command_gps(gps_string,2);
                    
                iterate++;
                
                
            }
        }
        else if (navigate_type == 5)
        {
            rover_command = "ctrl,-.50,-.50";
            message_motors.data = rover_command;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_motors.data.c_str());
            publisher_motors->publish(message_motors);
            usleep(3.0 * microsecond);

            rover_command = "ctrl,0.0,0.0";
            message_motors.data = rover_command;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_motors.data.c_str());
            publisher_motors->publish(message_motors);
            usleep(1.5 * microsecond);


            rover_command = "auto,turningTo,15000,180";
            message_motors.data = rover_command;
            publisher_motors->publish(message_motors);
            usleep(15.0 * microsecond);
        }
        
        // Rectilinear search pattern, mostly for SAR filming
        else if (navigate_type == 6)
        {
            std::cout << "Selected Square Search Pattern" << std::endl;
            message_motors.data = "led_set,303,0,0";
            publisher_motors->publish(message_motors);
            RCLCPP_INFO(this->get_logger(), "Begining Search");
            //
            for (int j = 0; j < gps_lat_target; j++)
            {
                //Rover faces North
                message_motors.data = "auto,turningTo,15000,350";
                publisher_motors->publish(message_motors);
                usleep(5 * microsecond);

                //Begins driving forward at 40% speed
                message_motors.data = "ctrl,-0.4,-0.4";
                publisher_motors->publish(message_motors);
                //Rover waits for the third input seconds before stopping
                usleep(gps_long_target * microsecond);
                message_motors.data = "ctrl,0.0,0.0";
                publisher_motors->publish(message_motors);
                //Rover faces East
                message_motors.data = "auto,turningTo,15000,80";
                publisher_motors->publish(message_motors);
                usleep(5 * microsecond);

                //Rover moves forward for a second before stopping againt
                message_motors.data = "ctrl,-0.4,-0.4";
                publisher_motors->publish(message_motors);
                usleep(4 * microsecond);
                message_motors.data = "ctrl,0.0,0.0";
                publisher_motors->publish(message_motors);
                //Rover faces South and drives that way for the same amount of time before stopping
                message_motors.data = "auto,turningTo,15000,170";
                publisher_motors->publish(message_motors);
                usleep(5 * microsecond);
                message_motors.data = "ctrl,-0.4,-0.4";
                publisher_motors->publish(message_motors);
                usleep(gps_long_target * microsecond);
                message_motors.data = "ctrl,0.0,0.0";
                publisher_motors->publish(message_motors);
                //Faces East again, moves forward one second, and stops the loop
                message_motors.data = "auto,turningTo,15000,80";
                publisher_motors->publish(message_motors);
                usleep(5 * microsecond);
                message_motors.data = "ctrl,-0.4,-0.4";
                publisher_motors->publish(message_motors);
                usleep(4 * microsecond);
                message_motors.data = "ctrl,0.0,0.0";
                publisher_motors->publish(message_motors);
            }
        }
        else if (navigate_type == 0)
        {
            message_motors.data = "ctrl,0,0";
            RCLCPP_INFO(this->get_logger(), "Stopping");
            publisher_motors->publish(message_motors);
        }
        else if (navigate_type == 7)
        {
            std::cout << "Looking For ARCUO" << std::endl;
            int cameraNum;
            std::cin >> cameraNum;
            cv::VideoCapture inputVideo(cameraNum);
            
            
            //inputVideo.open(cameraNum);
            cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
            cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            cv::aruco::ArucoDetector detector(dictionary, detectorParams);



            //cv::Size S = cv::Size((int) inputVideo.get(cv::CAP_PROP_FRAME_WIDTH),    // Acquire input size
              //  (int) inputVideo.get(cv::CAP_PROP_FRAME_HEIGHT));

            //cv::VideoWriter writer;
            //int codec = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
            //double fps = 15.0;
            //std::string filename = "footage.mp4";
            //cv::Size sizeFrame(640,480);
            //writer.open(filename, codec, fps, sizeFrame, true);
            //cv::Mat cameraMatrix, distCoeffs;
            //float markerLength = 0.05;

            //readCameraParameters(cameraParamsFilename, cameraMatrix, distCoeffs);

            //cv::Mat objPoints(4, 1, CV_32FC3);
            //objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
            //objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
            //objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
            //objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

            cv::Mat image, imageCopy;
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners, rejected;
            inputVideo >> image;
            std::cout << "Video Prepared" << std::endl;


            cv::Mat res;
            std::vector<cv::Mat> spl;
            cv::VideoWriter outputVideo;    
            int codec = cv::VideoWriter::fourcc('H', '2', '6', '4');  // select desired codec (must be available at runtime)
            double fps = 25.0;                          // framerate of the created video stream
            std::string filename = "./live.mp4";             // name of the output video file
            outputVideo.open(filename, codec, fps, image.size(), true);
            // check if we succeeded
            if (!outputVideo.isOpened()) {
                std::cerr << "Could not open the output video file for write\n";
                
                }



            std::cout << "Output prepared" << std::endl;

            int iterateIT = 0;
            while (inputVideo.grab()) 
                {
                iterateIT ++;
                cv::Mat image, imageCopy;
                inputVideo.retrieve(image);
                image.copyTo(imageCopy);
                //std::vector<int> ids;
                //std::vector<std::vector<cv::Point2f>> corners, rejected;
                detector.detectMarkers(image, corners, ids, rejected);
                // if at least one marker detected
                if (ids.size() > 0)
                    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

                //int nMarkers = corner.size();
                //std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

                //Calculate pose for each marker
                //for (int i = 0; i < nMarkers; i++)
                //{
                //   solvePnP(objPoints, corner.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                //}

                //Draw Axis for each marker
                //for (unsigned int i = 0; i < ids.size(); i++)
                //{
                //    cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1)
                //}

                outputVideo.write(imageCopy);


                sensor_msgs::CompressedImage::SharedPtr compressed_msg = cv_bridge::CvImage(hdr, "bgr8", imageCopy).toCompressedImageMsg()
                image_publisher.publish(compressed_msg);
                cv::imshow("out", imageCopy);
                
                //inputVideo >> imageCopy;
                //cv::Mat xframe;
                //resize(imageCopy, xframe, sizeFrame);
                //writer.write(xframe);


                //split(src, spl);                // process - extract only the correct channel
                //for (int i =0; i < 3; ++i)
                //{
                    
                //spl[i] = cv::Mat::zeros(S, spl[0].type());
                //    
                //}
                //merge(spl, res);

               // outputVideo.write(imageCopy);
                //outputVideo.write(res); //save or
                //outputVideo << res;

                //int waitTime = 1;
                char key = (char) cv::waitKey(1);
                if (key == 27)
                //std::cout << iterateIT << std::endl;
                //if (iterateIT >= 90)
                {
                    break;
                }
                
                

                }
               
            //bool isSuccess = imwrite("./MyImage.jpg", imageCopy); //write the image to a file as JPEG 
            //bool isSuccess = imwrite("D:/MyImage.png", image); //write the image to a file as PNG
            //if (isSuccess == false)
            //{
            //std::cout << "Failed to save the image" << std::endl;
            //std::cin.get(); //wait for a key press
            //}

            std::cout << "Finished filming!" << std::endl;
            inputVideo.release();
            //writer.release();
        }   
        else if (navigate_type == 8)
        {
             // Get the message describing the goal
            auto feedback_msg = std::make_shared<NavigateRover::Feedback>();
            // 
            auto &status_message = feedback_msg->current_status;

            status_message = 99;


            // Send some data back to the goal
            for (int i = 0; i < 2; i++) goal_handle->publish_feedback(feedback_msg);

            usleep(5000);
            if (rclcpp::ok()) {
            // Publish the result
            auto result = std::make_shared<NavigateRover::Result>();
            result->final_result = 10;

            
        }
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
    
    
    void topic_callback(const std_msgs::msg::String & msg) 
    {
        std::string command;
        command = msg.data;
        RCLCPP_INFO(this->get_logger(), "Recieved: '%s'", msg.data.c_str());
        

        
        std::string delimiter = ",";
            size_t pos = 0;
            std::string token;
            std::string token1;
            std::string scommand = command.c_str();
            pos = scommand.find(delimiter);
            token = scommand.substr(0, pos);
            

        if (token == "orientation")
        {
            RCLCPP_INFO(this->get_logger(), "Recieved IMU bearing");
            //Turns command into the proper bearing
            //imu_bearing = imu_command(command); 
        }
        else if (token == "gps")
        {
            RCLCPP_INFO(this->get_logger(), "Recieved GPS location");
            //Turns command into GPS string
            gps_string = command;
        }

    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_motors;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_feedback;
    //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    size_t count_;
    rclcpp_action::Server<NavigateRover>::SharedPtr navigate_rover_server_;
};

//*************************************************************************************************
// Main
//*************************************************************************************************
int main(int argc, char **argv)
{
    //Generates AruCo tags
    cv::Mat markerImage;
    cv::aruco::Dictionary dictionary1 = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::generateImageMarker(dictionary1, 1, 200, markerImage, 1);
    cv::imwrite("marker1.png", markerImage);

    //Camera stuff for OpenCV
    



    rclcpp::init(argc, argv);
    auto node1 = std::make_shared<NavigateRoverServerNode>(); 
    auto node2 = std::make_shared<NavigateRoverSubscriberNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node1);
    executor.add_node(node2);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}