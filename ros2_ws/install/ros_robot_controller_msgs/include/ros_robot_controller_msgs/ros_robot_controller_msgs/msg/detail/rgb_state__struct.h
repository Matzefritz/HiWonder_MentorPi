// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_robot_controller_msgs:msg/RGBState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/rgb_state.h"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__RGB_STATE__STRUCT_H_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__RGB_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/RGBState in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__msg__RGBState
{
  int32_t index;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} ros_robot_controller_msgs__msg__RGBState;

// Struct for a sequence of ros_robot_controller_msgs__msg__RGBState.
typedef struct ros_robot_controller_msgs__msg__RGBState__Sequence
{
  ros_robot_controller_msgs__msg__RGBState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__msg__RGBState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__RGB_STATE__STRUCT_H_
