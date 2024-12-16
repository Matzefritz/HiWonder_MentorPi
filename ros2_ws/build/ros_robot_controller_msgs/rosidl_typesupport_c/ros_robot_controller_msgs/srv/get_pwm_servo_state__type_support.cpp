// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from ros_robot_controller_msgs:srv/GetPWMServoState.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__struct.h"
#include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__type_support.h"
#include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace ros_robot_controller_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetPWMServoState_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPWMServoState_Request_type_support_ids_t;

static const _GetPWMServoState_Request_type_support_ids_t _GetPWMServoState_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetPWMServoState_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetPWMServoState_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetPWMServoState_Request_type_support_symbol_names_t _GetPWMServoState_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros_robot_controller_msgs, srv, GetPWMServoState_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_robot_controller_msgs, srv, GetPWMServoState_Request)),
  }
};

typedef struct _GetPWMServoState_Request_type_support_data_t
{
  void * data[2];
} _GetPWMServoState_Request_type_support_data_t;

static _GetPWMServoState_Request_type_support_data_t _GetPWMServoState_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetPWMServoState_Request_message_typesupport_map = {
  2,
  "ros_robot_controller_msgs",
  &_GetPWMServoState_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetPWMServoState_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetPWMServoState_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetPWMServoState_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPWMServoState_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_type_hash,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_type_description,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace ros_robot_controller_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ros_robot_controller_msgs, srv, GetPWMServoState_Request)() {
  return &::ros_robot_controller_msgs::srv::rosidl_typesupport_c::GetPWMServoState_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__struct.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__type_support.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ros_robot_controller_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetPWMServoState_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPWMServoState_Response_type_support_ids_t;

static const _GetPWMServoState_Response_type_support_ids_t _GetPWMServoState_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetPWMServoState_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetPWMServoState_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetPWMServoState_Response_type_support_symbol_names_t _GetPWMServoState_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros_robot_controller_msgs, srv, GetPWMServoState_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_robot_controller_msgs, srv, GetPWMServoState_Response)),
  }
};

typedef struct _GetPWMServoState_Response_type_support_data_t
{
  void * data[2];
} _GetPWMServoState_Response_type_support_data_t;

static _GetPWMServoState_Response_type_support_data_t _GetPWMServoState_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetPWMServoState_Response_message_typesupport_map = {
  2,
  "ros_robot_controller_msgs",
  &_GetPWMServoState_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetPWMServoState_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetPWMServoState_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetPWMServoState_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPWMServoState_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_type_hash,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_type_description,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace ros_robot_controller_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ros_robot_controller_msgs, srv, GetPWMServoState_Response)() {
  return &::ros_robot_controller_msgs::srv::rosidl_typesupport_c::GetPWMServoState_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__struct.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__type_support.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ros_robot_controller_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetPWMServoState_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPWMServoState_Event_type_support_ids_t;

static const _GetPWMServoState_Event_type_support_ids_t _GetPWMServoState_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetPWMServoState_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetPWMServoState_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetPWMServoState_Event_type_support_symbol_names_t _GetPWMServoState_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros_robot_controller_msgs, srv, GetPWMServoState_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_robot_controller_msgs, srv, GetPWMServoState_Event)),
  }
};

typedef struct _GetPWMServoState_Event_type_support_data_t
{
  void * data[2];
} _GetPWMServoState_Event_type_support_data_t;

static _GetPWMServoState_Event_type_support_data_t _GetPWMServoState_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetPWMServoState_Event_message_typesupport_map = {
  2,
  "ros_robot_controller_msgs",
  &_GetPWMServoState_Event_message_typesupport_ids.typesupport_identifier[0],
  &_GetPWMServoState_Event_message_typesupport_symbol_names.symbol_name[0],
  &_GetPWMServoState_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetPWMServoState_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPWMServoState_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_type_hash,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_type_description,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace ros_robot_controller_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ros_robot_controller_msgs, srv, GetPWMServoState_Event)() {
  return &::ros_robot_controller_msgs::srv::rosidl_typesupport_c::GetPWMServoState_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace ros_robot_controller_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _GetPWMServoState_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPWMServoState_type_support_ids_t;

static const _GetPWMServoState_type_support_ids_t _GetPWMServoState_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetPWMServoState_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetPWMServoState_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetPWMServoState_type_support_symbol_names_t _GetPWMServoState_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros_robot_controller_msgs, srv, GetPWMServoState)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_robot_controller_msgs, srv, GetPWMServoState)),
  }
};

typedef struct _GetPWMServoState_type_support_data_t
{
  void * data[2];
} _GetPWMServoState_type_support_data_t;

static _GetPWMServoState_type_support_data_t _GetPWMServoState_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetPWMServoState_service_typesupport_map = {
  2,
  "ros_robot_controller_msgs",
  &_GetPWMServoState_service_typesupport_ids.typesupport_identifier[0],
  &_GetPWMServoState_service_typesupport_symbol_names.symbol_name[0],
  &_GetPWMServoState_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetPWMServoState_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPWMServoState_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &GetPWMServoState_Request_message_type_support_handle,
  &GetPWMServoState_Response_message_type_support_handle,
  &GetPWMServoState_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    ros_robot_controller_msgs,
    srv,
    GetPWMServoState
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    ros_robot_controller_msgs,
    srv,
    GetPWMServoState
  ),
  &ros_robot_controller_msgs__srv__GetPWMServoState__get_type_hash,
  &ros_robot_controller_msgs__srv__GetPWMServoState__get_type_description,
  &ros_robot_controller_msgs__srv__GetPWMServoState__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace ros_robot_controller_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, ros_robot_controller_msgs, srv, GetPWMServoState)() {
  return &::ros_robot_controller_msgs::srv::rosidl_typesupport_c::GetPWMServoState_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
