//***********************************************
//rover-Autonomy server client
//Sends instructions to the server
//Last edited May 23, 2024
//Version: 1.3
//***********************************************
//Maintained by: Daegan Brown
//Number: 423-475-4384
//Email: daeganbrown03@gmail.com
//***********************************************
#include <iostream>
#include <vector>
#include <chrono>
#include <unistd.h>    

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "astra_auto_interfaces/action/navigate_rover.hpp"

using NavigateRover = astra_auto_interfaces::action::NavigateRover;
using NavigateRoverGoalHandle = rclcpp_action::ClientGoalHandle<NavigateRover>;
using namespace std::placeholders;

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/calib3d.hpp>
//#include <opencv2/aruco_samples_utlity.hpp>
//#include <opencv2/objdetect/aruco_dictionary.hpp>
using namespace std::chrono_literals;



class NavigateRoverClientNode : public rclcpp::Node 
{
public:
    NavigateRoverClientNode() : Node("navigate_rover_client") 
    {
        navigate_rover_client_ = 
            rclcpp_action::create_client<NavigateRover>(this, "navigate_rover");
    }

    void send_goal(int navigate_type, double gps_lat_target, 
        double gps_long_target, double target_radius, double period)
    {
        //Wait for the Action Server
        navigate_rover_client_->wait_for_action_server();

        // Create a goal
        auto goal = NavigateRover::Goal();
        goal.navigate_type = navigate_type;
        goal.gps_lat_target = gps_lat_target;
        goal.gps_long_target = gps_long_target;
        goal.target_radius = target_radius;
        goal.period = period;

        // Add callbacks
        auto options = rclcpp_action::Client<NavigateRover>::SendGoalOptions();
        options.feedback_callback =
            std::bind(&NavigateRoverClientNode::feedback_callback, this, _1, _2);
        options.result_callback = 
            std::bind(&NavigateRoverClientNode::goal_result_callback, this, _1);
        options.goal_response_callback =
            std::bind(&NavigateRoverClientNode::goal_response_callback, this, _1);


        //*****************************************************************************************

        //*****************************************************************************************

        // Cancel Previous Goal
        //count_until_client_->async_cancel_goal(goal_handle_);
        // NavigateRoverClientNode->async_cancel_all_goals();
        
    
        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending a goal");
        navigate_rover_client_->async_send_goal(goal, options);

        //Cancel the gaol, testing
        std::cout << "About to send Goal Handle" << std::endl;
        
        std::cout << "Sent Goal Handle" << std::endl;
        // timer_ = this->create_wall_timer(
        //     std::chrono::seconds(2),
        //     std::bind(&NavigateRoverClientNode::timer_callback, this)
        // );
    }

    void feedback_callback(
    NavigateRoverGoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateRover::Feedback> feedback)
    {
        std::stringstream ss;
    ss << "Next number in sequence received: ";
    RCLCPP_INFO(this->get_logger(), "%li", feedback->current_status);
    }
private:
    
    void timer_callback()
    {
        // navigate_rover_client_->async_cancel_goal(goal_handle_);
        timer_->cancel();
    }
    //Accepted or rejected
    void goal_response_callback(const NavigateRoverGoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Goal got rejected");
        }
        else
        {
            this->goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal got accepted");
            usleep(10000);
            navigate_rover_client_->async_cancel_goal(goal_handle_);
        }
    }
    // Callback to receive the results once the goal is done
    void goal_result_callback(const NavigateRoverGoalHandle::WrappedResult &result)
    {
        int final_result = result.result->final_result;
        RCLCPP_INFO(this->get_logger(), "Result: %d", final_result);
    }

    rclcpp_action::Client<NavigateRover>::SharedPtr navigate_rover_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    NavigateRoverGoalHandle::SharedPtr goal_handle_;

};
//*************************************************************************************************
//MAIN
//*************************************************************************************************

int main(int argc, char **argv)
{
    //*********************************************************************************************
    //Variables
    //*********************************************************************************************
    int navigate_type;      //Always will be 0, assigned later for clarity
    // double gps_lat_target;  //Actually the y-coordinates of the detected square
    // double gps_long_target; //x-coordinates of the detected square
    int x_coord = 0;
    int x2_coord = 0;
    int x3_coord = 0;
    int x4_coord = 0;

    int y_coord = 0;
    int y2_coord = 0;
    int y3_coord = 0;
    int y4_coord = 0;

    //*********************************************************************************************
    //OpenCV Aruco Shenanigans
    //*********************************************************************************************
    
    std::cout << "Starting Aruco Detection" << std::endl;
            // int cameraNum = 0;
            //std::cin >> cameraNum;
            cv::VideoCapture inputVideo("/dev/video0");
            cv::Mat camMatrix, distCoeffs;
            
            
            //inputVideo.open(cameraNum);
            cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
            cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            cv::aruco::ArucoDetector detector(dictionary, detectorParams);
            
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
            // int x_coord = 0;
            int debug_iterator = 0;
            while (inputVideo.grab()) 
                {
                iterateIT ++;
                std::cout << "Attempt " << iterateIT << std::endl;
                cv::Mat image, imageCopy;
                inputVideo.retrieve(image);
                
                cv::resize(image, imageCopy, cv::Size(640, 480), 0, 0, cv::INTER_AREA);
                //cv::namedWindow("out", CV_WINDOW_AUTOSIZE);
                //std::vector<int> ids;
                //std::vector<std::vector<cv::Point2f>> corners, rejected;
                detector.detectMarkers(imageCopy, corners, ids, rejected);
                // if at least one marker detected
                // int debug_iterator = 0;
                if (ids.size() > 0)
                {
                    /*
                    int Xdebug_aruco = (int)rejected[0][0].x;
                    int Ydebug_aruco = (int)rejected[0][0].y;
                    std::cout << '{' << Xdebug_aruco << ',' << Ydebug_aruco << '}' << std::endl;
                    int Xids = (int)ids[0];
                    std::cout << Xids << std::endl;
                    */
                    
                    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
                    std::cout << "Aruco Detected" << std::endl;
                    x_coord = (int)corners[0][0].x;
                    y_coord = (int)corners[0][0].y;

                    x2_coord = (int)corners[0][1].x;
                    y2_coord = (int)corners[0][1].y;

                    x3_coord = (int)corners[0][2].x;
                    y3_coord = (int)corners[0][2].y;

                    x4_coord = (int)corners[0][3].x;
                    y4_coord = (int)corners[0][3].y;


                    std::cout << '(' << x_coord << ',' << y_coord << ')' << std::endl;
                    std::cout << '(' << x2_coord << ',' << y2_coord << ')' << std::endl;
                    std::cout << '(' << x3_coord << ',' << y3_coord << ')' << std::endl;
                    std::cout << '(' << x4_coord << ',' << y4_coord << ')' << std::endl;
                    
                    /*
                    for (int i = 0; i < corners.size(); i++)
                    {
                        for (int j = 0; j < corners[i].size(); j++)
                        {
                            x_coord = (int)corners[i][j].x;
                            std::cout << x_coord << ' ';
                        }
                    }*/

                    
                    
                    debug_iterator++;
                    
                     
                    
                 }
                 else 
                 {
                    debug_iterator = 0;
                 }
                
                outputVideo.write(imageCopy);



                
                //cv::imshow("out", imageCopy);
                
                // char key = (char) cv::waitKey(1);
                if (debug_iterator >= 2)
                    break;
                
                
                
                }
               
          
            std::cout << "Finished filming!" << std::endl;
            inputVideo.release();
            std::cout << "Calculating Range" << std::endl;


            float pixelHeight = y4_coord - y_coord;
            float actualHeight = .15;
            float distance = 2.7432;
            float pixelWidth = x2_coord - x_coord;
            float actualWidth = .15;
            float focalH = pixelHeight * (distance/actualHeight);   // 597.408 for laptop
            float focalW = pixelWidth * (distance/actualWidth);     // 603.504 for laptop
            float meanFocal = (focalW + focalH)/2;
            std::cout << focalH << " is the focal length using height" << std::endl;
            std::cout << focalW << " is the focal length using width" << std::endl;
            std::cout << meanFocal << " is the mean focal length" << std::endl;

            float setMeanFocal = (457.2 + 475.488)/2;
            float distanceFromH = (457.2/pixelHeight) * actualHeight; 
            float distanceFromW = (475.488/pixelWidth) * actualWidth;
            float distanceFromMeanH = (setMeanFocal/pixelHeight) * actualHeight;
            float distanceFromMeanW = (setMeanFocal/pixelWidth) * actualWidth;
            float meanDistance = (distanceFromMeanH + distanceFromMeanW)/2;

            std::cout << distanceFromH << " is the distance from last focal H" << std::endl;
            std::cout << distanceFromW << " is the distance from last focal W" << std::endl; 
            std::cout << distanceFromMeanH << " is the distance from last focal H using mean focal" << std::endl; 
            std::cout << distanceFromMeanW << " is the distance from last focal W using mean focal" << std::endl; 
            std::cout << meanDistance << " is the distance using mean focal and mean from H and W" << std::endl; 
            //*************************************************************************************
            // Test 1
            // Set distance of 9 feet(2.7432 m)
            // Focal length(H, W, Mean) = (457.2, 475.488, 466.344)
            // 
            // Test 2
            // Set distance of 
            // Focal length(H, W, Mean) = ()
            //
            // Test 3
            // Set Focal length(H, W, Mean) of (457.2, 475.488, 466.344)
            // Distance (H, W, MH, MW, Mean) = ()
            // Measured Distance: 
            //
            // Test 4
            // Set Focal length(H, W, Mean) of ()
            // Distance (H, W, MH, MW, Mean) = ()



    //*********************************************************************************************
    //Outputs
    //*********************************************************************************************
    //Cancels current goal
    
    
    
    


    //Sends new goal request
    navigate_type = 10; 
    

    
    
    




    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateRoverClientNode>(); 
    node->send_goal(navigate_type, x_coord, x2_coord, distanceFromW, 0.8);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}