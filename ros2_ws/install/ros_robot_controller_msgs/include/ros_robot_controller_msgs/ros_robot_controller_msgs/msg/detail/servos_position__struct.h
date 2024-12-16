// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_robot_controller_msgs:msg/ServosPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/servos_position.h"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SERVOS_POSITION__STRUCT_H_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SERVOS_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'position'
#include "ros_robot_controller_msgs/msg/detail/servo_position__struct.h"

/// Struct defined in msg/ServosPosition in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__msg__ServosPosition
{
  double duration;
  ros_robot_controller_msgs__msg__ServoPosition__Sequence position;
} ros_robot_controller_msgs__msg__ServosPosition;

// Struct for a sequence of ros_robot_controller_msgs__msg__ServosPosition.
typedef struct ros_robot_controller_msgs__msg__ServosPosition__Sequence
{
  ros_robot_controller_msgs__msg__ServosPosition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__msg__ServosPosition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SERVOS_POSITION__STRUCT_H_
