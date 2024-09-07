cmake_minimum_required(VERSION 3.8)
project(auto_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(clucky_interfaces REQUIRED)

add_executable(astra_auto_server src/server.cpp)
ament_target_dependencies(astra_auto_server rclcpp rclcpp_action clucky_interfaces)


install(TARGETS  
  astra_auto_server
  DESTINATION lib/${PROJECT_NAME}/
  )

ament_package()
