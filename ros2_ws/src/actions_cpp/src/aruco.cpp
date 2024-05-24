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
        std::cout << "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFUUUUUUUUUUUUUUUUUUUU-" << std::endl;
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



                //cv::imshow("out", imageCopy);
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
        //Send final message
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