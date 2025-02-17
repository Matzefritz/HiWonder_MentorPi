# Workspace Directory
After the initial setup your `~/workspace` directory should look like this:
```bash
|-- workspace
    |-- ros2_ws
        |-- build
        |-- install
        |-- log
        |-- src
            |-- calibration
            |-- imu_calib
            |-- ldrobot-lidar-ros2
            |-- peripherals
            |-- driver
            |-- orchestrator_launch
            |-- simulations
    |-- software
        |-- Servo_upper_computer
```
This chapter provides an overwiew over the different folders and what they implement respectively.

## Software 
### Servo_upper_computer
This folder contains the source files for the gui based trimming tool for the servos the camera is attached to. 

## ros2_ws
This folder contains the ROS2 workspace. It follows the standard structure for a ROS2 workspace. For more information about the creation, structure and guidelines of a ROS2 workspace, see: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html .

### build
The `build` folder ist created during the build process. It contains intermediate files and metadata required for compiling and linking the packages in your workspace.
You may want to remove this folder for a clean subsequent build. 

### install
The `install` folder is created during the build process. It ontains all the files necessary to run the compiled ROS 2 packages, including executables, shared libraries, configuration files, and resources.

This folder also contains your source script, e.g. `local_setup.bash`. This needs to be sourced in order for ROS2 to set up up the environment variables required for ROS2 to recognize the packages and their dependencies in your workspace.  
You may choose to source this file every time a terminal is started by adding the source command to your .bashrc file.  
Additionally you may also want to source the file again after you added or removed packages to or from your workspace.

### log
The `log` folder contains all logging information from the build process, runtime, and launch events.

### src
The `src` folder contains all the source code from your workspace. It is usually organized into different packages that implement different functionality.


# Preinstalled ROS2 Packages
After completion of this repositorys README, the following packages are already installed. In this chapter, the functionality of each package is explained.

## calibration
## imu_calib
## ldrobot-lidar-ros2 -> ldlidar_node
This package handles the communication with the LIDAR. It is based on this repository:
https://github.com/Myzhar/ldrobot-lidar-ros2  
The actual ROS2 package lies in the `ldlidar_node` folder.  
The `ldlidar.yaml` in `ldlidar_node/params` folder defines important hardware dependent variables, such as the used serial port and the exact model of the LIDAR.
The `ldlidar_node` (the node, not the package. For this package the node name and the package name are identical) can be launched with a launch file provided in the `launch` folder of the package. Multiple launch files are provided, altough only `ldlidar.launch.py` has been tested.
 

## peripherals
## driver
## orchestrator_launch
## simulations


# IMU Calibration
The IMU is factory calibrated, however in our setup it is overwritten and replaced with a default file. Using the other calibration file works but isn't ideal.
The IMU can be calibrated with the following steps. First, launch the controller:
```bash
ros2 launch ros_robot_controller ros_robot_controller.launch.py
```
Then, start the IMU calibration:
```bash
ros2 run imu_calib do_calib --ros-args -r imu:=/ros_robot_controller/imu_raw --param output_file:=/home/[robot_name]/workspace/ros2_ws/src/calibration/config/imu_calib.yaml
```
Make sure to use the name of your robot. The IMU calibration tool will prompt you to align the robot in different orientations.
The orientations are slightly different from those typically used for robots (the x and y axis are switched).
![Robot Orientation](/images/robot_orientation.png)
