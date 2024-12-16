// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_robot_controller_msgs:msg/BusServoState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/bus_servo_state.h"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUS_SERVO_STATE__STRUCT_H_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUS_SERVO_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'present_id'
// Member 'target_id'
// Member 'position'
// Member 'offset'
// Member 'voltage'
// Member 'temperature'
// Member 'position_limit'
// Member 'voltage_limit'
// Member 'max_temperature_limit'
// Member 'enable_torque'
// Member 'save_offset'
// Member 'stop'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/BusServoState in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__msg__BusServoState
{
  rosidl_runtime_c__uint16__Sequence present_id;
  rosidl_runtime_c__uint16__Sequence target_id;
  rosidl_runtime_c__uint16__Sequence position;
  rosidl_runtime_c__int16__Sequence offset;
  rosidl_runtime_c__uint16__Sequence voltage;
  rosidl_runtime_c__uint16__Sequence temperature;
  rosidl_runtime_c__uint16__Sequence position_limit;
  rosidl_runtime_c__uint16__Sequence voltage_limit;
  rosidl_runtime_c__uint16__Sequence max_temperature_limit;
  rosidl_runtime_c__uint16__Sequence enable_torque;
  rosidl_runtime_c__uint16__Sequence save_offset;
  rosidl_runtime_c__uint16__Sequence stop;
} ros_robot_controller_msgs__msg__BusServoState;

// Struct for a sequence of ros_robot_controller_msgs__msg__BusServoState.
typedef struct ros_robot_controller_msgs__msg__BusServoState__Sequence
{
  ros_robot_controller_msgs__msg__BusServoState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__msg__BusServoState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUS_SERVO_STATE__STRUCT_H_
