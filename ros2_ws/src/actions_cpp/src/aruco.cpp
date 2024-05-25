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
        options.feedback_callback =
            std::bind(&NavigateRoverClientNode::feedback_callback, this, _1, _2);
        options.result_callback = 
            std::bind(&NavigateRoverClientNode::goal_result_callback, this, _1);

        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending a goal");
        navigate_rover_client_->async_send_goal(goal, options);
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

    // Callback to receive the results once the goal is done
    void goal_result_callback(const NavigateRoverGoalHandle::WrappedResult &result)
    {
        int final_result = result.result->final_result;
        RCLCPP_INFO(this->get_logger(), "Result: %d", final_result);
    }

    rclcpp_action::Client<NavigateRover>::SharedPtr navigate_rover_client_;
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
    double gps_lat_target;  //Actually the y-coordinates of the detected square
    double gps_long_target; //x-coordinates of the detected square
    int x_coord = 0;
    int x2_coord = 0;

    //*********************************************************************************************
    //OpenCV Aruco Shenanigans
    //*********************************************************************************************
    
    std::cout << "Starting Aruco Detection" << std::endl;
            int cameraNum = 0;
            //std::cin >> cameraNum;
            cv::VideoCapture inputVideo(cameraNum);
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
            while (inputVideo.grab()) 
                {
                iterateIT ++;
                int x_coord = 0;
                cv::Mat image, imageCopy;
                inputVideo.retrieve(image);
                
                cv::resize(image, imageCopy, cv::Size(640, 480), 0, 0, cv::INTER_AREA);
                //cv::namedWindow("out", CV_WINDOW_AUTOSIZE);
                //std::vector<int> ids;
                //std::vector<std::vector<cv::Point2f>> corners, rejected;
                detector.detectMarkers(image, corners, ids, rejected);
                // if at least one marker detected
                if (ids.size() > 0)
                {
                    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
                    std::cout << "Aruco Detected" << std::endl;
                    x_coord = (int)corners[0][0].x;
                    x2_coord = (int)corners[0][1].x;
                    std::cout << '(' << x_coord << ',' << x2_coord << ')' << std::endl;
                    
                    /*
                    for (int i = 0; i < corners.size(); i++)
                    {
                        for (int j = 0; j < corners[i].size(); j++)
                        {
                            x_coord = (int)corners[i][j].x;
                            std::cout << x_coord << ' ';
                        }
                    }*/

                 }
                
                outputVideo.write(imageCopy);



                
                // cv::imshow("out", imageCopy);
                
                char key = (char) cv::waitKey(1);
                if (x_coord != 0)
                
                {
                    break;
                }
                
                
                }
               
          
            std::cout << "Finished filming!" << std::endl;
            inputVideo.release();



    //*********************************************************************************************
    //Outputs
    //*********************************************************************************************
    //Cancels current goal
    //HOW


    //Sends new goal request
    navigate_type = 10; 
    

    
    
    




    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateRoverClientNode>(); 
    node->send_goal(navigate_type, x_coord, x2_coord, 0.8);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}