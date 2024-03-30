# rover-Autonomy
# UAH ASTRA Team Autonomy

This project is designed for the 2024 URC competition, specifically for the autonomy portion. It was made by the UAH ASTRA Software team, with the hopes to build a foundation for future challenges as well. 

# Table of Contents
1. Title
2. Table of Requirements
3. Requirments
   - ROS2 Humble
   - Colcon
   - OpenCV
   - Git
4. Recommended Programs
5. How to Use
6. Common Problems
7. Author 
8. Maintainer

# Requirments 
Before downloading anything, it is a good idea to make sure your system is up to date and that all software you are using is as well. If you are on Ubuntu/Debian, the following commands will do so. 

All commands in this document will be for Ubuntu.

```
sudo apt-get install && sudo apt-get upgrade
```

## ROS2 Humble
We use ROS2(Robot Operating System) Humble, and plan on using that until support for it expires. If you are not using Ubuntu/Debian, go to the [ROS2 Website](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to know how to install ROS2, otherwise follow along.

The first step is to make sure you have Ubuntu Universe Repository, which is neccesary for ROS.

```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

After those are installed, you need to add the ROS2 GPG key through apt.

```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add that repository to your sources list. 

```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Now to actually install ROS2 Humble

```
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

Now ROS2 has been downloaded. Remember, to actually use ROS in a terminal, you have to source it with `source /opt/ros/humble/setup.bash`. You can also add this line to the bottom of your .bashrc file if you want to always have ros2 sourced in your terminal.

## Colcon
Colcon is used for building ros2 packages, and is quick to install. The following command is all that you need if you are running Ubuntu/Debian, otherwise check the [ROS2 Website](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for information on how to download.

```
sudo apt install python3-colcon-common-extension
```

## OpenCV

## Git


# Recommended Programs

## VSCode
VSCode is a wonderful program. I used it to make this project and reccomend anyone else working on it use it as well. 

```
sudo snap install --classic code
```

# How to Use
The purpose of this program is to, as stated above, run the ASTRA Rover autonomously within the [URC](https://urc.marssociety.org/home/about-urc) guidlines for the autonomous mission, made first for the 2024 mission, but modular to fit future missions.

There are two parts to running this project, one is to run the server, and the other is to run the client. Once the server is started, you do not need to touch it again. The client, however, is what is used to communicate with the server. A new command requires a new client.

The Server is run with the command

```
ros2 run actions_cpp navigate_rover_server
```

The Client is run with the command

```
ros2 run actions_cpp navigate_rover_client
```

Once the client has been started, it will ask you what input type you would like to use. This is the type of action you wish the rover to perform. The options are listed below. After the input type, there will be two more inputs, which are parameters for the function, which are listed below as well in the format (First input, Second input). 

1. Go to GPS Point
   - (Target Latitude, Target Longitude)
2. Go and search around point for ARUCO
   - (Target Latitude, Target Longitude)
3. GO and search around point for objects
   - (Target Latitude, Target Longitude)
4. Same as 1, but without looping, for test
   - (Target Latitude, Target Longitude)
5. Drives forward a small amount
   - Does nothing
6. Search Pattern
   - Does nothing
7. ARUCO Test
   - Does nothing
8. Object Detection
   - Does nothing


# Common Problems


# Author

Name: Daegan M Brown

Email: daeganbrown03@gmail.com

Phone: 423-475-4384

# Maintainer

Currently also Daegan. See above