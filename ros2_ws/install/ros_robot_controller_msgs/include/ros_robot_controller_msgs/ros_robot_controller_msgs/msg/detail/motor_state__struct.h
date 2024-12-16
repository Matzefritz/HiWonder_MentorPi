// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_robot_controller_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/motor_state.h"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/MotorState in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__msg__MotorState
{
  uint16_t id;
  double rps;
} ros_robot_controller_msgs__msg__MotorState;

// Struct for a sequence of ros_robot_controller_msgs__msg__MotorState.
typedef struct ros_robot_controller_msgs__msg__MotorState__Sequence
{
  ros_robot_controller_msgs__msg__MotorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__msg__MotorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
