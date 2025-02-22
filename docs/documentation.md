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
The `install` folder is created during the build process. It contains all the files necessary to run the compiled ROS 2 packages, including executables, shared libraries, configuration files, and resources.

This folder also contains your source script, e.g. `local_setup.bash`. This needs to be sourced in order for ROS2 to set up up the environment variables required for ROS2 to recognize the packages and their dependencies in your workspace.  
You may choose to source this file every time a terminal is started by adding the source command to your .bashrc file.  
Additionally you may also want to source the file again after you added or removed packages to or from your workspace and built the workspace with `colcon build`.

### log
The `log` folder contains all logging information from the build process, runtime, and launch events.

### src
The `src` folder contains all the source code from your workspace. It is usually organized into different packages that implement different functionality.

### How to make changes to your code:
Edit your code in `src`, then navigate to `/workspace/ros2_ws` (you have to navigate here to build properly) and enter `colcon build`, then source the workspace with `source install/local_setup.bash`. 

# Preinstalled ROS2 Packages
After completion of this repositorys README, the following packages are already installed. The source code and structure of the packages is provided by Hiwonder. It may be a bit confusing, so here is a small explanation of the functionality of each package. 

## calibration
This package contains the calibration files for the IMU, the launch files and python scripts for the angular and linear velocity calibration. The calibration file for the angular and linear velocity calibration is stored under `driver/controller/config/calibrate_params.yaml`. 

## driver
This package contains the code necessary to control the robot. The `controller.launch.py` launch file used in the setup is located in `/driver/controller/launch`. 
This file basically starts up the robot: 
1. **`odom_publisher.launch.py`** - First, the file `odom_publisher.launch.py` (in `driver/controller/launch`) is launched, which then starts the following three files
- **`robot_description.launch.py`** - The robot description is found under `/simulations/mentorpi_description` and contains the URDF (Unified Robot Description Format) files for the robot. URDF is an XML-based format used to describe the kinematic and dynamic properties of a robot, including its links, joints, sensors, and visual representation. The files are in xacro format, which is an XML-based macro language that extends URDF. A `/robot_description` topic is launched which can be used to view the robot in rviz. This also starts the `/tf` and `/tf_static` topics, which publish the transforms.
- **`ros_robot_controller.launch.py`** - This launches the `ros_robot_controller_node`, which is responsible for most functions of the robot: it publishes the raw IMU data, sets the led, buzzer, motors, servos, etc.
- **`odom_publisher_node`** - the `odom_publisher_node` calculates the robot's position and orientation based on velocity commands and then publishes raw odometry data on the `/odom_raw` topic.
2. **`imu_filter.launch.py`**
- this launches another launch file under `peripherals/launch`, which first applies the IMU calibration and then starts an IMU filter node. The raw IMU data is filtered with the `imu_complementary_filter` package (http://wiki.ros.org/imu_complementary_filter) and published on the `/imu` topic. 
3. **`ekf_filter_node`**
- An extended Kalman filter node is started through the `robot_localization` (http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) package

## imu_calib
This package contains the code required to run the IMU calibration. More info on how to calibrate the IMU can be found in the [IMU calibration section][#imu-calibration-instructions].

## ldrobot-lidar-ros2 -> ldlidar_node
This package handles the communication with the LIDAR. It is based on this repository:
https://github.com/Myzhar/ldrobot-lidar-ros2  
The actual ROS2 package lies in the `ldlidar_node` folder.  
The `ldlidar.yaml` in `ldlidar_node/params` folder defines important hardware dependent variables, such as the used serial port and the exact model of the LIDAR.
The `ldlidar_node` (the node, not the package. For this package the node name and the package name are identical) can be launched with a launch file provided in the `launch` folder of the package. Multiple launch files are provided, altough only `ldlidar.launch.py` has been tested.

## orchestrator_launch
This package contains the files to launch SLAM. The `slam_toolbox.launch.py` starts up the SLAM algorithm and `launch_full_slam_stack.launch.py` launches this file along with the controller, lidar, camera and joystick. 

## peripherals


## simulations


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
