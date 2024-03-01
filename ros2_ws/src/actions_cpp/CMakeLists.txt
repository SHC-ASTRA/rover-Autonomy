cmake_minimum_required(VERSION 3.8)
project(actions_cpp VERSION 1.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(astra_auto_interfaces REQUIRED)

find_package( OpenCV REQUIRED )


include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( /usr/local/include/opencv4/opencv2/objdetect/)
link_libraries( ${OpenCV_LIBS} )
link_libraries( ${OpenCV_INCLUDE_DIRS} )

add_executable(count_until_server src/count_until_server.cpp)
ament_target_dependencies(count_until_server rclcpp rclcpp_action astra_auto_interfaces)

add_executable(count_until_client src/count_until_client.cpp)
ament_target_dependencies(count_until_client rclcpp rclcpp_action astra_auto_interfaces)

add_executable(navigate_rover_server src/astra_auto_server.cpp)
ament_target_dependencies(navigate_rover_server rclcpp rclcpp_action std_msgs astra_auto_interfaces)
target_link_libraries( navigate_rover_server ${OpenCV_LIBS} )

add_executable(navigate_rover_client src/astra_auto_client.cpp)
ament_target_dependencies(navigate_rover_client rclcpp rclcpp_action astra_auto_interfaces)





install(TARGETS 
  count_until_server
  count_until_client
  navigate_rover_server
  navigate_rover_client
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
