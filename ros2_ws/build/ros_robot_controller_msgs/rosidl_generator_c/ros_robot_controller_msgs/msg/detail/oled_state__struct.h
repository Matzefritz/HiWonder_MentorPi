// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_robot_controller_msgs:msg/OLEDState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/oled_state.h"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__OLED_STATE__STRUCT_H_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__OLED_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'text'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/OLEDState in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__msg__OLEDState
{
  uint8_t index;
  rosidl_runtime_c__String text;
} ros_robot_controller_msgs__msg__OLEDState;

// Struct for a sequence of ros_robot_controller_msgs__msg__OLEDState.
typedef struct ros_robot_controller_msgs__msg__OLEDState__Sequence
{
  ros_robot_controller_msgs__msg__OLEDState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__msg__OLEDState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__OLED_STATE__STRUCT_H_
