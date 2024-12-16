// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_robot_controller_msgs:msg/SetBusServoState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/set_bus_servo_state.h"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SET_BUS_SERVO_STATE__STRUCT_H_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SET_BUS_SERVO_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'state'
#include "ros_robot_controller_msgs/msg/detail/bus_servo_state__struct.h"

/// Struct defined in msg/SetBusServoState in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__msg__SetBusServoState
{
  ros_robot_controller_msgs__msg__BusServoState__Sequence state;
  double duration;
} ros_robot_controller_msgs__msg__SetBusServoState;

// Struct for a sequence of ros_robot_controller_msgs__msg__SetBusServoState.
typedef struct ros_robot_controller_msgs__msg__SetBusServoState__Sequence
{
  ros_robot_controller_msgs__msg__SetBusServoState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__msg__SetBusServoState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SET_BUS_SERVO_STATE__STRUCT_H_
