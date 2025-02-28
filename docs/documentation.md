# Workspace Directory
After the initial setup, your `~/workspace` directory should look like this:
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
```
This chapter provides an overwiew over the different folders and files and what they implement respectively.

<!--
## Software 
### Servo_upper_computer
This folder contains the source files for the gui based trimming tool for the servos the camera is attached to. 
-->

# ros2_ws
This folder contains the ROS2 workspace. It follows the standard structure for a ROS2 workspace. For more information about the creation, structure and guidelines of a ROS2 workspace, see: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html .

## build
The `build` folder ist created during the build process. It contains intermediate files and metadata required for compiling and linking the packages in your workspace.
You may want to remove this folder for a clean subsequent build. 

## install
The `install` folder is created during the build process. It contains all the files necessary to run the compiled ROS 2 packages, including executables, shared libraries, configuration files, and resources.

This folder also contains your source script, e.g. `local_setup.bash`. This needs to be sourced in order for ROS2 to set up up the environment variables required for ROS2 to recognize the packages and their dependencies in your workspace.  
You may choose to source this file every time a terminal is started by adding the source command to your .bashrc file.  
You may also need to source the file again after you added or removed packages to or from your workspace and built the workspace with `colcon build`.

## log
The `log` folder contains all logging information from the build process, runtime, and launch events.

## src
The `src` folder contains the actual source code from your workspace. It is usually organized into different packages that implement different functionality.

# Preinstalled ROS2 Packages
After completion of this repository's README, the following packages are already installed. Most of these packages are provided by Hiwonder and where only slightly adapted to work under this project structure. Each package is introduced briefly:

## calibration
The package contains:
- the calibration files for the IMU
- the launch files and python scripts for the angular and linear velocity calibration (Note: the calibration file for the angular and linear velocity calibration is not stored in calibration, instead it is stored under `driver/controller/config/calibrate_params.yaml`) 

## driver
This folder contains the code necessary to communicated with Hiwonders extension board. This board handles the 4 motors, the IMU and the 2 camera servos. The code is structured like this:
```
|-- driver
    |-- controller
    |-- ros_robot_controller
    |-- ros_robot_controller_msgs
    |-- sdk
```
Each of the four subfolders are packages. The `/controller` package contains the files that are necessary for robot startup. The `/ros_robot_controller`, `/ros_robot_controller_msgs` and `/sdk` folders are used for communication with the extension board. You propably wont need to modify any of this functionality.

#

The `controller` package includes the `controller.launch.py` launch file (located in `/driver/controller/launch`). This file basically starts up the robot. Here is what it does: 
1. First, it launches the file `odom_publisher.launch.py` (in `driver/controller/launch`) is launched, which then starts the following three files
    
    1. `robot_description.launch.py` - (located in `/simulations/mentorpi_description`), this launches a `/robot_description` topic and the `/tf` and `/tf_static` topics, which publish the transforms.
    2. `ros_robot_controller.launch.py` - This launches the `ros_robot_controller_node`, which is responsible for most functions of the robot: it publishes the raw IMU data, sets the led, buzzer, motors, servos, etc. and communicates with the extension board
    3. `odom_publisher_node` - the `odom_publisher_node` calculates the robot's position and orientation based on velocity commands and then publishes raw odometry data on the `/odom_raw` topic.

2. Then, it launches `imu_filter.launch.py`, which launches another launch file under `peripherals/launch`. This launch file first applies the IMU calibration and then starts an IMU filter node. The raw IMU data is filtered with the `imu_complementary_filter` package (http://wiki.ros.org/imu_complementary_filter) and published on the `/imu` topic. 

3. Lastly, an `ekf_filter_node` is started, this node uses an extended Kalman filter is started through the `robot_localization` (http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) package

After launching `controller.launch.py`, all of the main functions of the robot should be running:
- robot_description should be published
- the robot should be able to receive velocity commands
- the communication to the extension board should be set up (through `ros_robot_controller`)
- the transforms should be published (`/tf` and `/tf_static`)
- there should be raw and filtered imu data
- there should be raw and filtered odometry data

## imu_calib
This package contains the code required to run the IMU calibration. More info on how to calibrate the IMU can be found in the [IMU calibration section](#imu-calibration-instructions). Note: the IMU calibration file is not saved in this folder, it is saved in the `calibration` package.

## ldrobot-lidar-ros2 -> ldlidar_node
This package handles the communication with the LiDAR sensor, the code is from this repository: https://github.com/Myzhar/ldrobot-lidar-ros2  
The actual ROS2 package lies in the `ldlidar_node` folder. The `ldlidar.yaml` file in the folder `ldlidar_node/params` defines important hardware dependent variables, such as the used serial port and the exact model of the LiDAR.
The node `ldlidar_node` (the package and node name are identical for this package) can be launched with a launch file provided in the `launch` folder of the package. Multiple launch files are provided, altough only `ldlidar.launch.py` has been tested.

## orchestrator_launch
This package contains the files to launch SLAM. The `slam_toolbox.launch.py` starts up the SLAM algorithm and `launch_full_slam_stack.launch.py` launches this file along with the controller, lidar, camera and joystick.

## peripherals
The peripherals package contains:
- code to use the joystick/teleop, can be launched via `joystick_control.launch.py` or `teleop_key_control.launch.py`
- code to use the camera, can be launched via `usb_cam.launch.py` 
- other code for debugging

## simulations
This package contains:
- the robot description in URDF (Unified Robot Description Format). URDF is an XML-based format used to describe the kinematic and dynamic properties of a robot, including its links, joints, sensors, and visual representation. The files are in xacro format, which is an XML-based macro language that extends URDF
- the STL files used for the robot model (can be used to display the robot as a 3D model in rviz)
- some launch files


# IMU Calibration Instructions
The IMU is factory calibrated, however in our setup it is overwritten and replaced with a default file. Using the other calibration file works but isn't ideal.
The IMU can be calibrated with the following steps. First, launch the controller:
```bash
ros2 launch ros_robot_controller ros_robot_controller.launch.py
```
Then, start the IMU calibration:
```bash
ros2 run imu_calib do_calib --ros-args -r imu:=/ros_robot_controller/imu_raw --param output_file:=/home/[robot_name]/workspace/ros2_ws/src/calibration/config/imu_calib.yaml
```
Make sure to use the name of your robot. 
The IMU calibration tool will prompt you to align the robot in different orientations.
The orientations are slightly different from those typically used by ROS2 (the x and y axis are switched).
![Robot Orientation](/images/robot_orientation.png)
At the end of the calibration, you can check if the calibration worked with:
```bash
ros2 launch peripherals imu_view.launch.py
```
More info can be found here: https://drive.google.com/drive/folders/1qu4A3Uby4zEpbni56ReNzecZyhYH9-jl 
