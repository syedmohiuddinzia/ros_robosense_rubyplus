# Robosense Ruby Plus setup on Robot OS
this repository contains the installation procedure of robosense ruby plus for robot os. </br>
For a better understanding of the RoboSense Ruby Plus RS ROS driver, please refer to the [Robosense Lidar - rsLidar SDK - Documentation](https://github.com/RoboSense-LiDAR/rslidar_sdk).

## Robosense Lidar Ruby Plus
The RoboSense LiDAR Ruby Plus is an advanced 128-beam LiDAR system designed to deliver high-resolution, 3D perception for autonomous vehicles and other applications requiring precise environmental mapping. Ruby Plus offers a wide field of view and long-range detection capabilities, making it suitable for diverse environments and challenging conditions. It boasts enhanced accuracy and reliability, leveraging RoboSense's cutting-edge MEMS (Micro-Electro-Mechanical Systems) technology to provide detailed, real-time data crucial for safe navigation and object recognition. With its robust performance, the Ruby Plus LiDAR aims to push the boundaries of autonomous technology, contributing significantly to the development of safer and more efficient autonomous systems.

## Official Documentation
To learn more about the RoboSense LiDAR Ruby Plus, please visit the [official documentation](https://www.robosense.ai/en/rslidar/RS-Ruby_Plus). This comprehensive resource provides detailed information on the Ruby Plus's features, setup, and usage, guiding you through everything you need to know to get started and make the most of your device. Whether you're configuring your LiDAR system, understanding its advanced capabilities, or troubleshooting, the documentation offers step-by-step instructions and helpful insights to ensure a smooth experience.

## Important Lidar Terms
| Term                       | Definition                                                                                     |
|----------------------------|------------------------------------------------------------------------------------------------|
| LiDAR                      | Light Detection and Ranging, a remote sensing method that uses laser light to measure distances.|
| Point Cloud                | A set of data points in space produced by LiDAR, representing the external surface of an object or area.|
| Field of View (FoV)        | The extent of the observable world seen at any given moment by the LiDAR sensor.               |
| Resolution                 | The detail level of the point cloud, determined by the number of points per unit area.         |
| Range                      | The maximum distance at which the LiDAR can accurately measure objects.                        |
| Beam                       | The laser pulse emitted by the LiDAR sensor.                                                   |
| Scanning Frequency         | The rate at which the LiDAR sensor emits laser pulses, measured in Hertz (Hz).                 |
| Accuracy                   | The degree to which the measured distance is close to the true distance.                       |
| Calibration                | The process of adjusting the LiDAR system to ensure accurate measurements.                     |
| Reflectivity               | The measure of how much laser light is reflected back to the LiDAR sensor from an object.      |
| Data Rate                  | The speed at which data is transmitted from the LiDAR sensor, typically measured in Mbps (Megabits per second).|
| MEMS                       | Micro-Electro-Mechanical Systems, technology used in some LiDAR sensors for beam steering.     |
| Time of Flight (ToF)       | The time it takes for the laser pulse to travel to an object and back to the sensor, used to calculate distance.|
| Multi-Echo                 | The capability of the LiDAR to detect multiple reflections from a single laser pulse.          |
| Sensor Fusion              | The process of combining data from multiple sensors to improve overall accuracy and reliability.|
| MSOP                       | Main data Stream Output Protocol.                                                              |
| DIFOP                      | Device Info Output Protocol.                                                                   |
| PTP                        | Precision Time Protocol, used for synchronizing clocks throughout a computer network.          |
| NTP                        | Network Time Protocol, used for clock synchronization between computer systems over packet-switched data networks.|
| GPS                        | Global Positioning System, a satellite-based navigation system.                                |
| UTC                        | Universal Time Coordinated, the primary time standard by which the world regulates clocks and time.|
| Protocol                   | Protocol version number, 01 represents the old version, 02 represents the latest version.      |
| Wave_mode                  | Echo flag, indicating the type of echo detected.                                               |
| Temp                       | Sensor temperature information.                                                                |
| Resv                       | Reserved data flag.                                                                            |
| Ret_id                     | Return ID in the data packet.                                                                  |
| Azimuth                    | LiDAR horizontal rotation angle.                                                               |
| Timestamp                  | Time stamp used to record system time.                                                         |
| Header                     | Frame header in protocol packet.                                                               |
| Tail                       | Frame tail in protocol packet.                                                                 |

## Initial Setup, of Operating System and RS Lidar
1. **Install Ubuntu 18.04 and ROS Melodic on Laptop:**
Install Ubuntu 18.04 LTS on your laptop if not already installed, You can find the installation guide [here](https://ubuntu.com/tutorials/install-ubuntu-desktop-1804#1-overview).
Then, follow the instructions to install ROS Melodic. You can find the installation guide [here](http://wiki.ros.org/melodic/Installation/Ubuntu).
2. **Install RSView V4.3.11:**
Install the RSView V4.3.11 on your Ubuntu 18.04 Operating System from [RoboSense Ruby Plus - Download Link](https://www.robosense.ai/en/resources-89). This app will be used to configure and monitor your Reach RS2 GNSS receiver. I have also uploaded the ubuntu 18.04 and windows 10-11 RS View on my [Google Drive](https://drive.google.com/drive/folders/1KrJ3JA4d_RNE5xUVY1MWcMSFLXd5MXkr?usp=sharing).
3. **Network Configuration for RS Lidar Ruby Plus:**
Connect the RS-Lidar to PC over Ethernet cable and power supply. The sensor has set the default IP address to computer at factory, therefore configure the **default IP address** of the computer to ```192.168.1.102```, **sub-net mask** to ```255.255.255.0```. Besides, users should make sure that the ***RSView doesn’t be blocked by any firewall or third party security software***.
4. **Startup RSView:**
   - Go to download directory of **RSView**
   - Open terminal and write
   ```bash run_rsview.sh``` </br>
      <img src="https://github.com/syedmohiuddinzia/ros_robosense_rubyplus/blob/main/images/1.png" alt="rosrun" width="750"/>
   - When RSView opens, then click on the **File** -> **Open** -> **Sensor Stream** </br>
      <img src="https://github.com/syedmohiuddinzia/ros_robosense_rubyplus/blob/main/images/2.png" alt="rosrun" width="750"/>
   - Configure the callibration values, and then **click OK**. </br>
      <img src="https://github.com/syedmohiuddinzia/ros_robosense_rubyplus/blob/main/images/3.png" alt="rosrun" width="500"/>
   - Leave **Group IP:** ```0.0.0.0``` and **Host IP:** ```0.0.0.0```, Configure the **MSOP Port:** ```6699``` and **DIFOP Port:** ```7788```. And then **click OK**. </br>
      <img src="https://github.com/syedmohiuddinzia/ros_robosense_rubyplus/blob/main/images/4.png" alt="rosrun" width="350"/>
   - Now you will see the stream of RS Lidar Ruby Plus. </br>
      <img src="https://github.com/syedmohiuddinzia/ros_robosense_rubyplus/blob/main/images/5.png" alt="rosrun" width="750"/>
            
## Setting up RS Lidar SDK on Robot OS
1. **Clone and Compiling ROS Package for RS Lidar SDK:**
   - Navigate to src directory catkin_ws directory and download or clone the directory from the GitHub repository [here](https://github.com/RoboSense-LiDAR/rslidar_sdk.git).
     ```bash
     cd ~/catkin_ws/src
     git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
     ```
   - Install dependency Yaml version >= v0.5.2 and libpcap version >= v1.7.4
      ```
      sudo apt-get update
      sudo apt-get install -y libyaml-cpp-dev
      sudo apt-get install -y  libpcap-dev
      ```
   - To compile using catkin_make method, open **CMakeLists.txt** and set the variable COMPILE_METHOD to CATKIN. </br>
      <img src="https://github.com/syedmohiuddinzia/ros_robosense_rubyplus/blob/main/images/6.png" alt="rosrun" width="500"/>
   - If you are setting up **RS Lidar Ruby Plus**, then open **config.yaml** file in **rslidar_sdk/config** directory. And write in **lidar_type:** ```RSP80```. </br>
      <img src="https://github.com/syedmohiuddinzia/ros_robosense_rubyplus/blob/main/images/7.png" alt="rosrun" width="750"/>      
   - For compilation got catkin_make directory and run commands
     ```
     catkin_make
     source devel/setup.bash
     ```
2. **Launch RSLidar_SDK:**
   - To open RS Lidar SDK and read stream of RS Ruby Plus run the command
      ```bash
      roslaunch rslidar_sdk start.launch
      ```
   - After ROS launching the RS-Lidar SDK, RVIZ will open with pointcloud data. The pointcloud topic /rslidar_points is published now. </br>
      <img src="https://github.com/syedmohiuddinzia/ros_robosense_rubyplus/blob/main/images/8.png" alt="rosrun" width="1200"/>

## Working
1. Write **rostopic list** on terminal. After running the above command following topics can be displayed on terminal
   ```
   /clicked_point
   /initialpose
   /move_base_simple/goal
   /rosout
   /rosout_agg
   /rslidar_points
   /tf
   /tf_static
   ```
   <img src="https://github.com/syedmohiuddinzia/ros_robosense_rubyplus/blob/main/images/9.png" alt="rosrun" width="250"/>
2. Write rostopic echo /rslidar_points on terminal. This is a point cloud data streaming from RS Lidar. After running the above command following topics can be displayed on terminal
```
*continued ... 213, 170, 26, 63, 241, 36, 121, 61, 182, 147, 45, 189, 0, 0, 128, 63, 35, 0, 189, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 44, 0, 189, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 45, 0, 198, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 46, 0, 198, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 47, 0, 198, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 40, 0, 198, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 41, 0, 207, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 42, 0, 207, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 43, 0, 207, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 52, 0, 207, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 53, 0, 216, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 54, 0, 216, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 55, 0, 216, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 48, 0, 216, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 49, 0, 225, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 50, 0, 225, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 51, 0, 225, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 60, 0, 225, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 61, 0, 235, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 62, 0, 235, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 63, 0, 235, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 56, 0, 235, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 57, 0, 245, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 58, 0, 245, 37, 193, 158, 174, 159, 217, 65, 18, 233, 21, 63, 219, 110, 111, 61, 244, 24, 144, 188, 0, 0, 128, 63, 59, 0, 245, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 68, 0, 245, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 69, 0, 254, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 70, 0, 254, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 71, 0, 254, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 64, 0, 254, 37, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 65, 0, 7, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 66, 0, 7, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 67, 0, 16, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 76, 0, 16, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 77, 0, 24, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 78, 0, 57, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 79, 0, 64, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 72, 0, 64, 38, 193, 158, 174, 159, 217, 65, 1, 168, 8, 63, 50, 200, 31, 188, 254, 69, 97, 187, 0, 0, 128, 63, 73, 0, 64, 38, 193, 158, 174, 159, 217, 65, 162, 125, 49, 63, 253, 81, 229, 60, 164, 149, 92, 187, 0, 0, 128, 63, 74, 0, 71, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 75, 0, 71, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 4, 0, 78, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 5, 0, 78, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 6, 0, 78, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 7, 0, 78, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 2, 0, 83, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 0, 0, 83, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 1, 0, 83, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 3, 0, 83, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 12, 0, 88, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 13, 0, 88, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 14, 0, 93, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 15, 0, 93, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 8, 0, 93, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 9, 0, 98, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 10, 0, 98, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 11, 0, 104, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 20, 0, 104, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 21, 0, 109, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 22, 0, 109, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 23, 0, 114, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 16, 0, 114, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 17, 0, 119, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 18, 0, 119, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 19, 0, 125, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 28, 0, 125, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 29, 0, 132, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 30, 0, 132, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 31, 0, 141, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 24, 0, 141, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 25, 0, 149, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 26, 0, 149, 38, 193, 158, 174, 159, 217, 65, 123, 47, 20, 63, 1, 2, 106, 61, 33, 33, 70, 189, 0, 0, 128, 63, 27, 0, 149, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 36, 0, 149, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 37, 0, 158, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 38, 0, 158, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 39, 0, 158, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 32, 0, 158, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 33, 0, 166, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 34, 0, 166, 38, 193, 158, 174, 159, 217, 65, 159, 39, 24, 63, 187, 154, 112, 61, 148, 149, 42, 189, 0, 0, 128, 63, 35, 0, 166, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 44, 0, 166, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 45, 0, 175, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 46, 0, 175, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 47, 0, 175, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 40, 0, 175, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 41, 0, 184, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 42, 0, 184, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 43, 0, 184, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 52, 0, 184, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 53, 0, 193, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 54, 0, 193, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 55, 0, 193, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 48, 0, 193, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 49, 0, 202, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 50, 0, 202, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 51, 0, 202, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 60, 0, 202, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 61, 0, 212, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 62, 0, 212, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 63, 0, 212, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 56, 0, 212, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 57, 0, 222, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 58, 0, 222, 38, 193, 158, 174, 159, 217, 65, 17, 193, 25, 63, 84, 140, 113, 61, 13, 245, 147, 188, 0, 0, 128, 63, 59, 0, 222, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 68, 0, 222, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 69, 0, 231, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 70, 0, 231, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 71, 0, 231, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 64, 0, 231, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 65, 0, 240, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 66, 0, 240, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 67, 0, 249, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 76, 0, 249, 38, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 77, 0, 1, 39, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 78, 0, 34, 39, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 79, 0, 41, 39, 193, 158, 174, 159, 217, 65, 180, 23, 26, 63, 254, 26, 67, 189, 47, 34, 161, 187, 0, 0, 128, 63, 72, 0, 41, 39, 193, 158, 174, 159, 217, 65, 59, 197, 13, 63, 65, 247, 53, 188, 244, 49, 106, 187, 0, 0, 128, 63, 73, 0, 41, 39, 193, 158, 174, 159, 217, 65, 255, 86, 53, 63, 25, 92, 224, 60, 101, 143, 97, 187, 0, 0, 128, 63, 74, 0, 48, 39, 193, 158, 174, 159, 217, 65, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 0, 0, 75, 0, 48, 39, 193, 158, 174, 159, 217, 65]
is_dense: False
---
```
   <img src="https://github.com/syedmohiuddinzia/ros_robosense_rubyplus/blob/main/images/10.png" alt="rosrun" width="1200"/>


