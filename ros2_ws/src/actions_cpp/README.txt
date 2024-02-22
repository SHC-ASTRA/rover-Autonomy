README for astra_auto_interfaces
Ros2 package containing the action server for ASTRA Rover autonomy.

Maintainer: Daegan Brown
    Email: daeganbrown03@gmail.com 
    Phone: 423-475-4384


Requirements
    astra_auto_interfaces 

Version History
    1.0 
        Simply ran a command to the rover to head forward. 
    1.1
        Asked user for GPS input 
        Able to subscribe to IMU topic
        Sends more feedback to console on what mode you've selected
    1.2
        Loops GPS locating, until within 0.0000001 lat and long of target.

Contents:
    directory: include
        Contains ???? probably CMake stuff ???
    directory: src
        Contains the source files. C++ and header files
    File: CMakeLists.txt
        This is the startup information for CMake when compiling the package
    File: packages.xml
        Helps with package startup, giving depencencies and libraries
