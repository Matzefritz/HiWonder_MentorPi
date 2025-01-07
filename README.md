# MentorPI mecanum-wheel development starter pack
This repository aims to provide a reasonable starting position for ROS2 development on the Raspberry Pi 5 based MentorPi robot platform from Hiwonder. Specifically the version equipped with the mecanum-wheel drivetrain and the gimbal monocular camera. See: https://www.hiwonder.com/collections/raspberrypi-bionic-robot/products/mentorpi-m1?variant=41285892702295

# General Information
The MentorPi plaform from Hiwonder is a Raspberry Pi 5 based robot platform. As the Raspberry Pi 5 runs normal linux in this setup, it can be thought of as a normal computer. It can therefore be used with a mouse, keyboard and a monitor as one would expect from a standard computer.
As a development framework, ROS2 is used. ROS2 is an open-source framework for building robotic applications. It acts as the middleware between the different components of the robot and also provides tools, libraries, hardware abstraction, device drivers and more for standardised robot development.
For more information, see the ROS2 documentation: https://docs.ros.org/en/jazzy/index.html

# Basic Setup
In this chapter the basic setup of the robot is explained. You will install Linux, ROS2 and all the necessary drivers for the motors, servos, camera and lidar.
Ubuntu 24.04 together with ROS2 Jazzy is used in this project.

## Linux Setup
First, Ubuntu 24.04 needs to be installed on the Raspberry Pi 5.

1. **Flash Linux Image**  
The easiest way to flash the linux image is via the Raspberry Pi imager.
* For windows you can download the program here: https://www.raspberrypi.com/software/
* On Ubuntu you can simply install the program via the following command:
```bash
sudo apt install rpi-imager
```
In the Raspberry Pi imager tool choose "Raspberry Pi 5" under device and under Choose OS: "Other general-purpose OS" -> "Ubuntu" -> "Ubuntu Desktop 24.04.1 LTS (64-bit)"". Then choose the micro sd card (at least 64GB) you want to install the OS on and click next to continue following the instructions of the tool.

2. **Boot for the first time**
* Put the SD card in the Raspberry Pi 5, connect mouse, keyboard and a monitor via a micro HDMI cable and boot the Raspberry Pi 5.
* Follow the installer for Ubuntu

## Installing ROS2
* Follow this guide: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html to install ROS2 Jazzy on your system. Chose the **Desktop Install**. 
* Add 
```bash
source /opt/ros/jazzy/setup.bash
```
to your .bashrc file in order to source the ROS2 workspace every time a terminal is started.

### Install Additional Dependencies for ROS2

* **navigation2**
   - A ROS 2 package for robot navigation, enabling autonomous path planning, obstacle avoidance, and control.
   - Required for setting up navigation tasks in your robotic project.

* **joint-state-publisher**
   - A tool for publishing the state of all joints in a robot.
   - Essential for visualizing robot movement or simulating joint positions in RViz.

* **xacro**
   - Stands for XML Macros and is used to simplify the creation of URDF files for robot models.
   - Required for generating dynamic robot description files.

* **ros-jazzy-imu-complementary-filter**
   - Provides a complementary filter for IMU data processing in the ROS Jazzy distribution.
   - Helps in fusing accelerometer and gyroscope data for smoother motion tracking.

* **python3-transforms3d**
   - A Python library for handling 3D transformations such as rotations and translations.
   - Used in the `JoystickControl` package for manipulating 3D poses and transformations.

* **python3-pydantic**
   - A Python library for data validation and settings management.
   - Required by the `ros usb_cam` package to handle camera configurations and data structures.

## SSH Setup
For easier development connecting to the Raspberry Pi 5 vie SSH is strongly recomended. For this, the Raspberry Pi 5 needs to be connected to the same network as the device from which you want to access the Raspberry Pi 5 (Eduroam does not work). Once this is made shure you can look up the IP address from the Raspberry Pi 5 with
```bash
hostname -I
```
Now you can connect to the Raspberry Pi 5 from your computer with
```bash
ssh user@IP-address
```
