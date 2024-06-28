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
      <img src="https://github.com/syedmohiuddinzia/ros_robosense_rubyplus/blob/main/images/1.png" alt="rosrun" width="500"/>
   - When RSView opens, then click on the **File** -> **Open** -> **Sensor Stream** </br>
      <img src="https://github.com/syedmohiuddinzia/ros_robosense_rubyplus/blob/main/images/2.png" alt="rosrun" width="750"/>
   - Configure the callibration values, and then **click OK**.
   - Leave **Group IP:** ```0.0.0.0``` and **Host IP:** ```0.0.0.0```, Configure the **MSOP Port:** ```6699``` and **DIFOP Port:** ```7788```. And then **click OK**.
   - Now you will see the stream of RS Lidar Ruby Plus.
  
## Setting up RS Lidar on Robot OS
5. **Clone and Catkin_make ROS Package:**
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
   - To compile using catkin_make method, open **CMakeLists.txt** and set the variable COMPILE_METHOD to CATKIN.
  
   - 
12. **Run ROS Core:**
Start the ROS core by running the following command in a terminal:
     ```bash
     roscore
     ```
13. **Run Reach RS2 ROS Driver Node:**
Launch the Reach RS2 ROS driver node with specific parameters for host/IP and port using the following command:
     ```bash
     rosrun reach_rs_driver reach_rs_driver_node _reach_rs_host_or_ip:=192.168.137.231 _reach_rs_port:=9001
     ```
14. **Ensure Reach RS2 is connected on ROS**
If everything is working fine, you will see
   ```
   [INFO] [1719069234.009935]: Connecting to 192.168.137.231:9001...
   [INFO] [1719069234.100768]: Successfully connected to device!
   ```
<img src="https://github.com/syedmohiuddinzia/ros_reach_rs2/blob/main/images/3.png" alt="rosrun" width="1200"/>
