// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_robot_controller_msgs:srv/GetPWMServoState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/srv/get_pwm_servo_state.h"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__SRV__DETAIL__GET_PWM_SERVO_STATE__STRUCT_H_
#define ROS_ROBOT_CONTROLLER_MSGS__SRV__DETAIL__GET_PWM_SERVO_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'cmd'
#include "ros_robot_controller_msgs/msg/detail/get_pwm_servo_cmd__struct.h"

/// Struct defined in srv/GetPWMServoState in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__srv__GetPWMServoState_Request
{
  ros_robot_controller_msgs__msg__GetPWMServoCmd__Sequence cmd;
} ros_robot_controller_msgs__srv__GetPWMServoState_Request;

// Struct for a sequence of ros_robot_controller_msgs__srv__GetPWMServoState_Request.
typedef struct ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence
{
  ros_robot_controller_msgs__srv__GetPWMServoState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'state'
#include "ros_robot_controller_msgs/msg/detail/pwm_servo_state__struct.h"

/// Struct defined in srv/GetPWMServoState in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__srv__GetPWMServoState_Response
{
  bool success;
  ros_robot_controller_msgs__msg__PWMServoState__Sequence state;
} ros_robot_controller_msgs__srv__GetPWMServoState_Response;

// Struct for a sequence of ros_robot_controller_msgs__srv__GetPWMServoState_Response.
typedef struct ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence
{
  ros_robot_controller_msgs__srv__GetPWMServoState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  ros_robot_controller_msgs__srv__GetPWMServoState_Event__request__MAX_SIZE = 1
};
// response
enum
{
  ros_robot_controller_msgs__srv__GetPWMServoState_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetPWMServoState in the package ros_robot_controller_msgs.
typedef struct ros_robot_controller_msgs__srv__GetPWMServoState_Event
{
  service_msgs__msg__ServiceEventInfo info;
  ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence request;
  ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence response;
} ros_robot_controller_msgs__srv__GetPWMServoState_Event;

// Struct for a sequence of ros_robot_controller_msgs__srv__GetPWMServoState_Event.
typedef struct ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence
{
  ros_robot_controller_msgs__srv__GetPWMServoState_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_ROBOT_CONTROLLER_MSGS__SRV__DETAIL__GET_PWM_SERVO_STATE__STRUCT_H_
