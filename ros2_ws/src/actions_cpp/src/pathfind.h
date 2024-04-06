//*************************************************************************************************
//rover-Autonomy Header file
//Holds functions for various pathfinding
//Last edited Feb 29, 2024
//*************************************************************************************************
//Maintained by: Daegan Brown
//Number: 423-475-4384
//Email: daeganbrown03@gmail.com
//*************************************************************************************************
//Includes
//*************************************************************************************************


// C++ includes
#include <memory>                           //
#include <chrono>                           //
#include <functional>                       //
#include <string>                           //
#include <unistd.h>                         // usleep
#include <iostream>                         // cout and cin
#include <cmath>                            // sin, cos, tan2,

//*************************************************************************************************
// Find Facing
//*************************************************************************************************
//When given current latitude/longitude, and a target latitude/longitude, this function
//will output a bearing to face the target point.
int find_facing(double gps_lat_target, double gps_long_target, \
                    double current_lat, double current_long) 
{
    std::cout << gps_lat_target << ", " << gps_long_target << std::endl;
    double X;
    double Y;
    double neededHeading;
    double deltaLong = gps_long_target - current_long;
    double deg2rad = (3.141592/180);
    double rad2deg = (180/3.141592);
    int i_neededHeading;

    X = ( std::cos(deg2rad * gps_lat_target) * std::sin(deg2rad * deltaLong));
    Y = ( std::cos(deg2rad * current_lat) * std::sin( deg2rad * gps_lat_target))\
        - (std::sin(deg2rad * current_lat) * std::cos(deg2rad * gps_lat_target) * \
        std::cos(deg2rad * deltaLong));
    neededHeading = (rad2deg * atan2(X,Y)) + 360;
    i_neededHeading = neededHeading;
    i_neededHeading = i_neededHeading % 360;
    std::cout << i_neededHeading << std::endl;
    return i_neededHeading;
}

//*************************************************************************************************
// Find Distance Remaining
//*************************************************************************************************
//Find out how far the rover has left to go, in meters. 
//Input is GPS targets then GPS currents
float find_distance(double gps_lat_target, double gps_long_target, \
    double current_lat, double current_long) 
{
            
            
    double deg2rad = (180.0/3.141592);
    double deltaLat = deg2rad * gps_lat_target - current_lat;
    double deltaLong = deg2rad * gps_long_target - current_long;
    double a;
    double c;
    double d;
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
//*************************************************************************************************
//Parses imu string for IMU facing
std::string orientation_string(std::string command)
{
            
    std::string delimiter = ",";
    size_t pos = 0;
    std::string token;
    std::string token1;
    std::string scommand = command.c_str();
    pos = scommand.find(delimiter);
    token = scommand.substr(0, pos);
            

    if (token == "orientation")
    {
        scommand.erase(0, pos + delimiter.length());
        //float command_r = std::stof(scommand);
        return scommand;
    }
    else //NEED FAILSTATE ASAP
    {
        return scommand;
    }

        }

//*************************************************************************************************
// Parse GPS string 
//*************************************************************************************************
//Parses imu string for GPS location, Lat or Long depending on parameter
double imu_command_gps(std::string command, int choose)
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
        scommand.erase(pos + delimiter.length(), (scommand.length()));
        std::cout << scommand << std::endl;  //remove for final 
        command = scommand;   
        double latitude = std::stod(scommand);
        std::cout << latitude;
        return latitude;
    }
    else if (choose == 2)
    {
        scommand.erase(0,pos + delimiter.length());
        pos = scommand.find(delimiter); 
        scommand.erase(pos  + delimiter.length(), (scommand.length() - 1));
        command = scommand;
        std::cout << scommand << std::endl;    //remove for final 
        double longitude = std::stod(scommand); 
        std::cout << longitude; 
        return longitude;
    }
    else //ADD FAILSTATE ASAP
    {
        return pos;
    }
            
}