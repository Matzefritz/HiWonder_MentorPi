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
First, Ubuntu 24.04 needs to be installed on the Raspberry Pi 5. The easiest way to achieve this is via the Raspberry Pi imager.
* For windows you can download the program here: https://www.raspberrypi.com/software/
* On Ubuntu you can simply install the program via the following command:
```bash
sudo apt install rpi-imager
```
