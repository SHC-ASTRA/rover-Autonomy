README for astra_auto_interfaces
Ros2 package containing the interface for communicating with ASTRA autonomy

Maintainer: Daegan Brown
    Email: daeganbrown03@gmail.com 
    Phone: 423-475-4384


Requirements
    actions_cpp server must be running to recieve goals

Contents:
    directory: action
        Contains the action files for the server
    directory: msg
        Contains the message files for the server
    directory: srv 
        Contains the server files for the server
    File: CMakeLists.txt
        This is the startup information for CMake when compiling the package
    File: packages.xml
        Helps with package startup, giving depencencies and libraries