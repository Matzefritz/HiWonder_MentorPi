// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from ros_robot_controller_msgs:srv/GetPWMServoState.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__functions.h"
#include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace ros_robot_controller_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetPWMServoState_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPWMServoState_Request_type_support_ids_t;

static const _GetPWMServoState_Request_type_support_ids_t _GetPWMServoState_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros_robot_controller_msgs, srv, GetPWMServoState_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros_robot_controller_msgs, srv, GetPWMServoState_Request)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPWMServoState_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_type_hash,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_type_description,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace ros_robot_controller_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Request>()
{
  return &::ros_robot_controller_msgs::srv::rosidl_typesupport_cpp::GetPWMServoState_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, ros_robot_controller_msgs, srv, GetPWMServoState_Request)() {
  return get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__functions.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ros_robot_controller_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetPWMServoState_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPWMServoState_Response_type_support_ids_t;

static const _GetPWMServoState_Response_type_support_ids_t _GetPWMServoState_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros_robot_controller_msgs, srv, GetPWMServoState_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros_robot_controller_msgs, srv, GetPWMServoState_Response)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPWMServoState_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_type_hash,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_type_description,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace ros_robot_controller_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Response>()
{
  return &::ros_robot_controller_msgs::srv::rosidl_typesupport_cpp::GetPWMServoState_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, ros_robot_controller_msgs, srv, GetPWMServoState_Response)() {
  return get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__functions.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ros_robot_controller_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetPWMServoState_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPWMServoState_Event_type_support_ids_t;

static const _GetPWMServoState_Event_type_support_ids_t _GetPWMServoState_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros_robot_controller_msgs, srv, GetPWMServoState_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros_robot_controller_msgs, srv, GetPWMServoState_Event)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPWMServoState_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_type_hash,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_type_description,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace ros_robot_controller_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Event>()
{
  return &::ros_robot_controller_msgs::srv::rosidl_typesupport_cpp::GetPWMServoState_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, ros_robot_controller_msgs, srv, GetPWMServoState_Event)() {
  return get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ros_robot_controller_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetPWMServoState_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPWMServoState_type_support_ids_t;

static const _GetPWMServoState_type_support_ids_t _GetPWMServoState_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros_robot_controller_msgs, srv, GetPWMServoState)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros_robot_controller_msgs, srv, GetPWMServoState)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPWMServoState_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<ros_robot_controller_msgs::srv::GetPWMServoState>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<ros_robot_controller_msgs::srv::GetPWMServoState>,
  &ros_robot_controller_msgs__srv__GetPWMServoState__get_type_hash,
  &ros_robot_controller_msgs__srv__GetPWMServoState__get_type_description,
  &ros_robot_controller_msgs__srv__GetPWMServoState__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace ros_robot_controller_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState>()
{
  return &::ros_robot_controller_msgs::srv::rosidl_typesupport_cpp::GetPWMServoState_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, ros_robot_controller_msgs, srv, GetPWMServoState)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState>();
}

#ifdef __cplusplus
}
#endif
