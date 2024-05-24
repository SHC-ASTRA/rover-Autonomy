#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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



/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ArucoDetector : public rclcpp::Node
{
  public:
    ArucoDetector()
    : Node("aruco_detect"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("/astra/auto/aruco", 10);

        std::cout << "Starting Aruco Detection" << std::endl;
            int cameraNum;
            std::cin >> cameraNum;
            cv::VideoCapture inputVideo(cameraNum);
            
            
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
                cv::Mat image, imageCopy;
                inputVideo.retrieve(image);
                image.copyTo(imageCopy);
                //std::vector<int> ids;
                //std::vector<std::vector<cv::Point2f>> corners, rejected;
                detector.detectMarkers(image, corners, ids, rejected);
                // if at least one marker detected
                if (ids.size() > 0)
                    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
                
                outputVideo.write(imageCopy);



                
                cv::imshow("out", imageCopy);
                
                char key = (char) cv::waitKey(1);
                if (key == 27)
                
                {
                    break;
                }
                
                
                }
               
          
            std::cout << "Finished filming!" << std::endl;
            inputVideo.release();
            


        auto message = std_msgs::msg::String();
        message.data = "Detecting";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);

      /*
      timer_ = this->create_wall_timer(
      500ms, std::bind(&ArucoDetector::timer_callback, this));
      */
    }

  private:
    /*void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }*/


    //rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetector>());
  rclcpp::shutdown();
  return 0;
}