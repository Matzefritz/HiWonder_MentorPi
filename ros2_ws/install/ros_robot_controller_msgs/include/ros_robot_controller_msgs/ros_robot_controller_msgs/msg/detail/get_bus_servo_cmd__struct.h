// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_robot_controller_msgs:msg/GetBusServoCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/get_bus_servo_cmd.h"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_BUS_SERVO_CMD__STRUCT_H_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_BUS_SERVO_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/GetBusServoCmd in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__msg__GetBusServoCmd
{
  uint8_t id;
  uint8_t get_id;
  uint8_t get_position;
  uint8_t get_offset;
  uint8_t get_voltage;
  uint8_t get_temperature;
  uint8_t get_position_limit;
  uint8_t get_voltage_limit;
  uint8_t get_max_temperature_limit;
  uint8_t get_torque_state;
} ros_robot_controller_msgs__msg__GetBusServoCmd;

// Struct for a sequence of ros_robot_controller_msgs__msg__GetBusServoCmd.
typedef struct ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence
{
  ros_robot_controller_msgs__msg__GetBusServoCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_BUS_SERVO_CMD__STRUCT_H_
