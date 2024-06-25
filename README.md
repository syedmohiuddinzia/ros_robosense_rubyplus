# ros_robosense_rubyplus
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

## Setup
1. **Install Ubuntu 18.04 and ROS Melodic on Laptop:**
Install Ubuntu 18.04 LTS on your laptop if not already installed, You can find the installation guide [here](https://ubuntu.com/tutorials/install-ubuntu-desktop-1804#1-overview).
Then, follow the instructions to install ROS Melodic. You can find the installation guide [here](http://wiki.ros.org/melodic/Installation/Ubuntu).
3. **Install RSView V4.3.11:**
Install the RSView V4.3.11 on your Ubuntu 18.04 Operating System from [RoboSense Ruby Plus - Download Link](https://www.robosense.ai/en/resources-89). This app will be used to configure and monitor your Reach RS2 GNSS receiver.
4. **Assemble Reach RS2 and Ensure Battery is Charged:**
Assemble your Emlid Reach RS2 GNSS receiver according to the manufacturer's instructions. Make sure the battery is fully charged before proceeding with setup.
5. **Setup for TCP Server Position Streaming:**
Configure the Reach RS2 for TCP server position streaming. Note down the IP address and port number that will be used for streaming the GNSS data.
6. **Find IP Address and Verify Port Using `ifconfig` and `nmap`:**
   - **Find IP Address using `ifconfig`:**
     Open a terminal on your Ubuntu laptop and type the following command to find your IP address:

     ```bash
     ifconfig
     ```
     Look for the `inet` section under your network interface (e.g., `eth0`, `wlan0`). Note down the IP address listed (e.g., `192.168.137.14`).
     Example output:
     ```
     flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.137.14  netmask 255.255.255.0  broadcast 192.168.137.255
        inet6 fe80::2989:acf5:ffce:77ea  prefixlen 64  scopeid 0x20<link>
        ether 30:24:32:43:3c:26  txqueuelen 1000  (Ethernet)
        RX packets 176808  bytes 206428950 (206.4 MB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 90543  bytes 15862196 (15.8 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
     ```
   - **Use `nmap` to Scan for Devices and Ports:**
     Open a new terminal and use `nmap` to scan the local network (`192.168.137.0/24` subnet in this example) and verify the open port on devices:
     ```bash
     nmap 192.168.137.0/24
     ```
     Replace `192.168.137.0/24` with your actual network subnet if different.
     Example output:
     ```
     Starting Nmap 7.60 ( https://nmap.org ) at 2024-06-22 19:59 PKT
     Nmap scan report for lenovo-P52s.mshome.net (192.168.137.14)
     Host is up (0.00013s latency).
     All 1000 scanned ports on lenovo-P52s.mshome.net (192.168.137.14) are closed
     
     Nmap scan report for Reach-Rover.mshome.net (192.168.137.231)
     Host is up (0.0093s latency).
     Not shown: 993 closed ports
     PORT     STATE SERVICE
     22/tcp   open  ssh
     80/tcp   open  http
     111/tcp  open  rpcbind
     1000/tcp open  cadlock
     2000/tcp open  cisco-sccp
     2010/tcp open  search
     9001/tcp open  tor-orport
     
     Nmap done: 256 IP addresses (2 hosts up) scanned in 5.27 seconds
     ```
     - **Interpretation:**
       - Ensure that the device `Reach-Rover.mshome.net` (IP: `192.168.137.231`) is detected.
       - Verify that port `9001/tcp` (tor-orport) is listed as open, confirming that the specified port is accessible on the device.
7. **Check Data Streaming Before Running ROS:**
Before starting ROS, verify that data is being streamed correctly from the Reach RS2. Open a terminal and use the `telnet` command to connect to the specified IP address and port. Enter:
     ```bash
     telnet 192.168.137.231 9001
     ```
     Replace `192.168.137.231` and `9001` with the actual IP address and port number configured for your Reach RS2. If successful, you should see streaming data indicating that the Reach RS2 GNSS receiver is transmitting position information over the TCP connection.
8. **Clone and Catkin_make ROS Package:**
Clone the ROS package for the Reach RS2 ROS driver from the GitHub repository [here](https://github.com/enwaytech/reach_rs_ros_driver/tree/master).
Navigate to your Catkin workspace and build the package using the following commands:
     ```bash
     cd ~/catkin_ws/src
     git clone https://github.com/enwaytech/reach_rs_ros_driver.git
     cd ~/catkin_ws
     catkin_make
     source devel/setup.bash
     ```
9. **Run ROS Core:**
Start the ROS core by running the following command in a terminal:
     ```bash
     roscore
     ```
10. **Run Reach RS2 ROS Driver Node:**
Launch the Reach RS2 ROS driver node with specific parameters for host/IP and port using the following command:
     ```bash
     rosrun reach_rs_driver reach_rs_driver_node _reach_rs_host_or_ip:=192.168.137.231 _reach_rs_port:=9001
     ```
11. **Ensure Reach RS2 is connected on ROS**
If everything is working fine, you will see
   ```
   [INFO] [1719069234.009935]: Connecting to 192.168.137.231:9001...
   [INFO] [1719069234.100768]: Successfully connected to device!
   ```
<img src="https://github.com/syedmohiuddinzia/ros_reach_rs2/blob/main/images/3.png" alt="rosrun" width="1200"/>
