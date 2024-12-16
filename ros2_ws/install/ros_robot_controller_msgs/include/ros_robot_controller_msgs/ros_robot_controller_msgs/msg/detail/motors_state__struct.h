// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_robot_controller_msgs:msg/MotorsState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/motors_state.h"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTORS_STATE__STRUCT_H_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTORS_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "ros_robot_controller_msgs/msg/detail/motor_state__struct.h"

/// Struct defined in msg/MotorsState in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__msg__MotorsState
{
  ros_robot_controller_msgs__msg__MotorState__Sequence data;
} ros_robot_controller_msgs__msg__MotorsState;

// Struct for a sequence of ros_robot_controller_msgs__msg__MotorsState.
typedef struct ros_robot_controller_msgs__msg__MotorsState__Sequence
{
  ros_robot_controller_msgs__msg__MotorsState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__msg__MotorsState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTORS_STATE__STRUCT_H_
