// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_robot_controller_msgs:msg/Sbus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/sbus.h"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SBUS__STRUCT_H_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SBUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'channel'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Sbus in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__msg__Sbus
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__float__Sequence channel;
} ros_robot_controller_msgs__msg__Sbus;

// Struct for a sequence of ros_robot_controller_msgs__msg__Sbus.
typedef struct ros_robot_controller_msgs__msg__Sbus__Sequence
{
  ros_robot_controller_msgs__msg__Sbus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__msg__Sbus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SBUS__STRUCT_H_
