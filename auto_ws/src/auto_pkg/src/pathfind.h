//*************************************************************************************************
//rover-Autonomy Header file
//Holds functions for various pathfinding
//Last edited September 7, 2024
//*************************************************************************************************
//Maintained by: Daegan Brown
//Number: 423-475-4384
//Email: daeganbrown03@gmail.com
//*************************************************************************************************
//Includes
//*************************************************************************************************

#pragma once

#include <memory>                           // 
#include <chrono>                           // 
#include <functional>                       // 
#include <string>                           // 
#include <unistd.h>                         // usleep
#include <iostream>                         // cout and cin
#include <cmath>                            // sin, cos, tan2,


//*************************************************************************************************
// Target Bearing
// Takes longitude/latitude target input and gives a needed bearing, in degrees.
//*************************************************************************************************

int target_bearing(double gps_lat_target, double gps_long_target, \
                    double current_lat, double current_long) 
{
    double X, Y, neededHeading, deltaLong, deg2rad, rad2deg;
    deltaLong = gps_long_target - current_long;
    deg2rad = (3.141592/180);
    rad2deg = (180/3.141592);
    int i_neededHeading;

    X = ( std::cos(deg2rad * gps_lat_target) * std::sin(deg2rad * deltaLong));
    Y = ( std::cos(deg2rad * current_lat) * std::sin( deg2rad * gps_lat_target))\
        - (std::sin(deg2rad * current_lat) * std::cos(deg2rad * gps_lat_target) * \
        std::cos(deg2rad * deltaLong));
    neededHeading = (rad2deg * atan2(X,Y)) + 360;
    i_neededHeading = neededHeading;
    i_neededHeading = i_neededHeading % 360;
    return i_neededHeading;
}

//*************************************************************************************************
// Find Distance Remaining
// Find distance from rover to target point, in meters. 
//*************************************************************************************************

float find_distance(double gps_lat_target, double gps_long_target, \
    double current_lat, double current_long) 
{      
    double deg2rad = (180.0/3.141592);
    double deltaLat = deg2rad * gps_lat_target - current_lat;
    double deltaLong = deg2rad * gps_long_target - current_long;
    double a, c, d;
    int R = 6371000;    //Earth Radius

    //Haversine formula
    a = (std::sin(deltaLat / 2) * std::sin(deltaLat / 2)) + \
        (std::cos(deg2rad * current_lat) * std::cos(deg2rad * gps_lat_target) * \
        (std::sin(deltaLong / 2) * std::sin(deltaLat / 2)));
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    d = R * c;
    return d;  
}

//*************************************************************************************************
// Parse IMU Facing String
// Parses imu string for IMU facing
//*************************************************************************************************

double parse_imu_facing(std::string command)
{
    std::string delimiter = ",";
    size_t pos = 0;
    std::string token;
    std::string token1;
    std::string scommand = command.c_str();
    double orientation;
    pos = scommand.find(delimiter);
    token = scommand.substr(0, pos);
            
    if (token == "orientation")
    {
        std::cout << "DEBUG: " << scommand << std::endl;
        scommand.erase(0, pos + delimiter.length());
        std::cout << "DEBUG: " << scommand << std::endl;
        //float command_r = std::stof(scommand);
        orientation = std::stod(scommand);
        return orientation;
    }
    else //NEED FAILSTATE ASAP
    {
        return 0;
    }
}


//*************************************************************************************************
// Parse GPS string 
// Parses imu string for GPS location, lat or long depending on parameter
//*************************************************************************************************

double parse_imu_gps(std::string command, int choose)
{
    std::string delimiter = ",";
    size_t pos = 0;
    std::string token;
    std::string scommand = command.c_str();
    pos = scommand.find(delimiter);
    scommand.erase(0, pos + delimiter.length());
    pos = scommand.find(delimiter);
    if (choose == 1)
    {
        std::cout << "DEBUG 1: " << scommand << std::endl;
        scommand.erase(pos + delimiter.length(), (scommand.length()));
        std::cout << "DEBUG 2: " << scommand << std::endl;
        command = scommand;   
        double latitude = std::stod(scommand);
        std::cout << latitude;
        return latitude;
    }
    else if (choose == 2)
    {
        std::cout << "DEBUG 3: " << scommand << std::endl;
        scommand.erase(0,pos + delimiter.length());
        std::cout << "DEBUG 4: " << scommand << std::endl;
        pos = scommand.find(delimiter); 
        //scommand.erase(pos  + delimiter.length(), (scommand.length() - 1));
        command = scommand;
        double longitude = std::stod(scommand); 
        std::cout << longitude; 
        return longitude;
    }
    else //ADD FAILSTATE ASAP
    {
        return pos;
    }
            
}

//*************************************************************************************************
// Set Rover Speed
// Takes right speed and left speed to set rover motor speeds. 
//*************************************************************************************************

void set_motors(float right, float left)
{
    
}