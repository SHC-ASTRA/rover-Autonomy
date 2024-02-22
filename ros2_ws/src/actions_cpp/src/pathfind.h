//***********************************************
//rover-Autonomy Header file
//Holds functions for various pathfinding
//Last edited Feb 21, 2024
//***********************************************
//Maintained by: Daegan Brown
//Number: 423-475-4384
//Email: daeganbrown03@gmail.com
//***********************************************
#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <unistd.h>
#include <iostream>
#include <cmath>


class pathfindFunctions {
    public: 
        
        //Find which direction the rover needs to face
        float find_facing(double gps_lat_target, double gps_long_target, 
            float currentHeading, double current_lat, double current_long) 
        {
            std::cout << gps_lat_target << ", " << gps_long_target << std::endl;
            double X;
            double Y;
            double neededHeading;
            double deltaLong = std::abs(gps_long_target - current_long);
            double deg2rad = (180.0/3.141592);

            X = ( std::cos(deg2rad * gps_lat_target) * std::sin(deg2rad * deltaLong));
            Y = ( std::cos(deg2rad * current_lat) * std::sin( deg2rad * gps_lat_target))\
                - (std::sin(deg2rad * current_lat) * std::cos(deg2rad * gps_lat_target) * std::cos(deg2rad * deltaLong));
            neededHeading = deg2rad * atan2(X,Y);
            std::cout << neededHeading << std::endl;
            return neededHeading;
        }

        //Find out how far the rover has left to go, in meters. 
        //Input is GPS targets then GPS currents
        float find_distance(double gps_lat_target, double gps_long_target, 
            float currentHeading, double current_lat, double current_long) 
        {
            
            
            double deg2rad = (180.0/3.141592);
            double deltaLat = deg2rad * std::abs(gps_lat_target - current_lat);
            double deltaLong = deg2rad * std::abs(gps_long_target - current_long);
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


        //Parses imu string for IMU facing
        std::string imu_command(std::string command)
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


        //Parses imu string for GPS location
        double imu_command_gps(std::string command, int choose)
        {
            std::string delimiter = ",";
            size_t pos = 0;
            std::string token;
            std::string scommand = command.c_str();
            pos = scommand.find(delimiter);
            if (choose == 1)
            {
                scommand.erase(pos + delimiter.length(), scommand.length());
                double latitude = std::stof(scommand);
                std::cout << latitude;
                return latitude;
            }
            else if (choose == 2)
            {
                scommand.erase(0,pos + delimiter.length());
                pos = scommand.find(delimiter);
                scommand.erase(pos  + delimiter.length(), scommand.length());
                double longitude = std::stof(scommand);
                std::cout << longitude;
                return longitude;
            }
            else //ADD FAILSTATE ASAP
            {
                return pos;
            }
            
        }
};