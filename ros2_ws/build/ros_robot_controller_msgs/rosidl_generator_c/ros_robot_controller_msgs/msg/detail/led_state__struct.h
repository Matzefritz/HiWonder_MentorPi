// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_robot_controller_msgs:msg/LedState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/led_state.h"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__LED_STATE__STRUCT_H_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__LED_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/LedState in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__msg__LedState
{
  uint8_t id;
  float on_time;
  float off_time;
  uint16_t repeat;
} ros_robot_controller_msgs__msg__LedState;

// Struct for a sequence of ros_robot_controller_msgs__msg__LedState.
typedef struct ros_robot_controller_msgs__msg__LedState__Sequence
{
  ros_robot_controller_msgs__msg__LedState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__msg__LedState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__LED_STATE__STRUCT_H_
