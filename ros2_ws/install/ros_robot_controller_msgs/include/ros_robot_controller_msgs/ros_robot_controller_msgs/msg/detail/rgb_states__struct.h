// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_robot_controller_msgs:msg/RGBStates.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/rgb_states.h"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__RGB_STATES__STRUCT_H_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__RGB_STATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'states'
#include "ros_robot_controller_msgs/msg/detail/rgb_state__struct.h"

/// Struct defined in msg/RGBStates in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__msg__RGBStates
{
  ros_robot_controller_msgs__msg__RGBState__Sequence states;
} ros_robot_controller_msgs__msg__RGBStates;

// Struct for a sequence of ros_robot_controller_msgs__msg__RGBStates.
typedef struct ros_robot_controller_msgs__msg__RGBStates__Sequence
{
  ros_robot_controller_msgs__msg__RGBStates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__msg__RGBStates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__RGB_STATES__STRUCT_H_
