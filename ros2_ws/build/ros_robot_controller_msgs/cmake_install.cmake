# Install script for directory: /home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ro01/workspace/ros2_ws/install/ros_robot_controller_msgs")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rosidl_interfaces" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/ros_robot_controller_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/BusServoState.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/ButtonState.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/BuzzerState.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/LedState.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/MotorState.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/MotorsState.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/PWMServoState.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/Sbus.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/OLEDState.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/SetBusServoState.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/RGBStates.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/RGBState.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/ServoPosition.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/ServosPosition.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/SetPWMServoState.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/GetBusServoCmd.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/msg/GetPWMServoCmd.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/srv" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/srv/GetBusServoState.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/srv" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_type_description/ros_robot_controller_msgs/srv/GetPWMServoState.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ros_robot_controller_msgs/ros_robot_controller_msgs" TYPE DIRECTORY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_c/ros_robot_controller_msgs/" REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/environment" TYPE FILE FILES "/opt/ros/jazzy/lib/python3.12/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/environment" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_generator_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_generator_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/libros_robot_controller_msgs__rosidl_generator_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_generator_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_generator_c.so"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_generator_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ros_robot_controller_msgs/ros_robot_controller_msgs" TYPE DIRECTORY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_typesupport_fastrtps_c/ros_robot_controller_msgs/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ros_robot_controller_msgs/ros_robot_controller_msgs" TYPE DIRECTORY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_cpp/ros_robot_controller_msgs/" REGEX "/[^/]*\\.hpp$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ros_robot_controller_msgs/ros_robot_controller_msgs" TYPE DIRECTORY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_typesupport_fastrtps_cpp/ros_robot_controller_msgs/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_cpp.so"
         OLD_RPATH "/opt/ros/jazzy/lib:/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_fastrtps_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ros_robot_controller_msgs/ros_robot_controller_msgs" TYPE DIRECTORY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_typesupport_introspection_c/ros_robot_controller_msgs/" REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/libros_robot_controller_msgs__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/libros_robot_controller_msgs__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_c.so"
         OLD_RPATH "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ros_robot_controller_msgs/ros_robot_controller_msgs" TYPE DIRECTORY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_typesupport_introspection_cpp/ros_robot_controller_msgs/" REGEX "/[^/]*\\.hpp$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_introspection_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/libros_robot_controller_msgs__rosidl_typesupport_introspection_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_introspection_cpp.so"
         OLD_RPATH "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_introspection_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/libros_robot_controller_msgs__rosidl_typesupport_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_cpp.so"
         OLD_RPATH "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_typesupport_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/environment" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/environment" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs-0.0.0-py3.12.egg-info" TYPE DIRECTORY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_python/ros_robot_controller_msgs/ros_robot_controller_msgs.egg-info/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs" TYPE DIRECTORY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_py/ros_robot_controller_msgs/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/ro01/workspace/ros2_ws/install/ros_robot_controller_msgs/lib/python3.12/site-packages/ros_robot_controller_msgs"
      )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs" TYPE MODULE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_py/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/ros_robot_controller_msgs_s__rosidl_typesupport_fastrtps_c.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs" TYPE MODULE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_py/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/ros_robot_controller_msgs_s__rosidl_typesupport_introspection_c.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs" TYPE MODULE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_generator_py/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_c.so"
         OLD_RPATH "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros_robot_controller_msgs/ros_robot_controller_msgs_s__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/ros_robot_controller_msgs_s__rosidl_typesupport_c.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_generator_py.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_generator_py.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/libros_robot_controller_msgs__rosidl_generator_py.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_generator_py.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_generator_py.so"
         OLD_RPATH "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libros_robot_controller_msgs__rosidl_generator_py.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/BusServoState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/ButtonState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/BuzzerState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/LedState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/MotorState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/MotorsState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/PWMServoState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/Sbus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/OLEDState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/SetBusServoState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/RGBStates.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/RGBState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/ServoPosition.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/ServosPosition.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/SetPWMServoState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/GetBusServoCmd.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/msg/GetPWMServoCmd.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/srv" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/srv/GetBusServoState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/srv" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_adapter/ros_robot_controller_msgs/srv/GetPWMServoState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/BusServoState.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/ButtonState.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/BuzzerState.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/LedState.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/MotorState.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/MotorsState.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/PWMServoState.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/Sbus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/OLEDState.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/SetBusServoState.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/RGBStates.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/RGBState.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/ServoPosition.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/ServosPosition.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/SetPWMServoState.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/GetBusServoCmd.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/msg" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/msg/GetPWMServoCmd.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/srv" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/srv/GetBusServoState.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/srv" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/srv/GetPWMServoState.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ros_robot_controller_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ros_robot_controller_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/environment" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/environment" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_index/share/ament_index/resource_index/packages/ros_robot_controller_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_generator_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_generator_cExport.cmake"
         "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/export_ros_robot_controller_msgs__rosidl_generator_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_generator_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_generator_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/export_ros_robot_controller_msgs__rosidl_generator_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/export_ros_robot_controller_msgs__rosidl_generator_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_typesupport_fastrtps_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_typesupport_fastrtps_cExport.cmake"
         "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/export_ros_robot_controller_msgs__rosidl_typesupport_fastrtps_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_typesupport_fastrtps_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_typesupport_fastrtps_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/export_ros_robot_controller_msgs__rosidl_typesupport_fastrtps_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/export_ros_robot_controller_msgs__rosidl_typesupport_fastrtps_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_generator_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_generator_cppExport.cmake"
         "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/export_ros_robot_controller_msgs__rosidl_generator_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_generator_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_generator_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/export_ros_robot_controller_msgs__rosidl_generator_cppExport.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_typesupport_fastrtps_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_typesupport_fastrtps_cppExport.cmake"
         "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/export_ros_robot_controller_msgs__rosidl_typesupport_fastrtps_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_typesupport_fastrtps_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_typesupport_fastrtps_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/export_ros_robot_controller_msgs__rosidl_typesupport_fastrtps_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/export_ros_robot_controller_msgs__rosidl_typesupport_fastrtps_cppExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_introspection_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_introspection_cExport.cmake"
         "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/ros_robot_controller_msgs__rosidl_typesupport_introspection_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_introspection_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_introspection_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/ros_robot_controller_msgs__rosidl_typesupport_introspection_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/ros_robot_controller_msgs__rosidl_typesupport_introspection_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_cExport.cmake"
         "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/ros_robot_controller_msgs__rosidl_typesupport_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/ros_robot_controller_msgs__rosidl_typesupport_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/ros_robot_controller_msgs__rosidl_typesupport_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_introspection_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_introspection_cppExport.cmake"
         "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/ros_robot_controller_msgs__rosidl_typesupport_introspection_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_introspection_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_introspection_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/ros_robot_controller_msgs__rosidl_typesupport_introspection_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/ros_robot_controller_msgs__rosidl_typesupport_introspection_cppExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_cppExport.cmake"
         "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/ros_robot_controller_msgs__rosidl_typesupport_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/ros_robot_controller_msgs__rosidl_typesupport_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/ros_robot_controller_msgs__rosidl_typesupport_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/ros_robot_controller_msgs__rosidl_typesupport_cppExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_generator_pyExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_generator_pyExport.cmake"
         "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/export_ros_robot_controller_msgs__rosidl_generator_pyExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_generator_pyExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake/export_ros_robot_controller_msgs__rosidl_generator_pyExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/export_ros_robot_controller_msgs__rosidl_generator_pyExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/CMakeFiles/Export/11fe27062861da08fc7608b6c6b48990/export_ros_robot_controller_msgs__rosidl_generator_pyExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_cmake/rosidl_cmake-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs/cmake" TYPE FILE FILES
    "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_core/ros_robot_controller_msgsConfig.cmake"
    "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ament_cmake_core/ros_robot_controller_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_robot_controller_msgs" TYPE FILE FILES "/home/ro01/workspace/ros2_ws/src/driver/ros_robot_controller_msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/ros_robot_controller_msgs__py/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/ro01/workspace/ros2_ws/build/ros_robot_controller_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
