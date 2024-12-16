// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_robot_controller_msgs:msg/ServoPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/servo_position.h"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SERVO_POSITION__STRUCT_H_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SERVO_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/ServoPosition in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__msg__ServoPosition
{
  uint16_t id;
  uint16_t position;
} ros_robot_controller_msgs__msg__ServoPosition;

// Struct for a sequence of ros_robot_controller_msgs__msg__ServoPosition.
typedef struct ros_robot_controller_msgs__msg__ServoPosition__Sequence
{
  ros_robot_controller_msgs__msg__ServoPosition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__msg__ServoPosition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SERVO_POSITION__STRUCT_H_
