// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ros_robot_controller_msgs:srv/GetPWMServoState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__functions.h"
#include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros_robot_controller_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetPWMServoState_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros_robot_controller_msgs::srv::GetPWMServoState_Request(_init);
}

void GetPWMServoState_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros_robot_controller_msgs::srv::GetPWMServoState_Request *>(message_memory);
  typed_message->~GetPWMServoState_Request();
}

size_t size_function__GetPWMServoState_Request__cmd(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ros_robot_controller_msgs::msg::GetPWMServoCmd> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetPWMServoState_Request__cmd(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ros_robot_controller_msgs::msg::GetPWMServoCmd> *>(untyped_member);
  return &member[index];
}

void * get_function__GetPWMServoState_Request__cmd(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ros_robot_controller_msgs::msg::GetPWMServoCmd> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetPWMServoState_Request__cmd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const ros_robot_controller_msgs::msg::GetPWMServoCmd *>(
    get_const_function__GetPWMServoState_Request__cmd(untyped_member, index));
  auto & value = *reinterpret_cast<ros_robot_controller_msgs::msg::GetPWMServoCmd *>(untyped_value);
  value = item;
}

void assign_function__GetPWMServoState_Request__cmd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<ros_robot_controller_msgs::msg::GetPWMServoCmd *>(
    get_function__GetPWMServoState_Request__cmd(untyped_member, index));
  const auto & value = *reinterpret_cast<const ros_robot_controller_msgs::msg::GetPWMServoCmd *>(untyped_value);
  item = value;
}

void resize_function__GetPWMServoState_Request__cmd(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ros_robot_controller_msgs::msg::GetPWMServoCmd> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetPWMServoState_Request_message_member_array[1] = {
  {
    "cmd",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros_robot_controller_msgs::msg::GetPWMServoCmd>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::srv::GetPWMServoState_Request, cmd),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetPWMServoState_Request__cmd,  // size() function pointer
    get_const_function__GetPWMServoState_Request__cmd,  // get_const(index) function pointer
    get_function__GetPWMServoState_Request__cmd,  // get(index) function pointer
    fetch_function__GetPWMServoState_Request__cmd,  // fetch(index, &value) function pointer
    assign_function__GetPWMServoState_Request__cmd,  // assign(index, value) function pointer
    resize_function__GetPWMServoState_Request__cmd  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetPWMServoState_Request_message_members = {
  "ros_robot_controller_msgs::srv",  // message namespace
  "GetPWMServoState_Request",  // message name
  1,  // number of fields
  sizeof(ros_robot_controller_msgs::srv::GetPWMServoState_Request),
  false,  // has_any_key_member_
  GetPWMServoState_Request_message_member_array,  // message members
  GetPWMServoState_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GetPWMServoState_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetPWMServoState_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetPWMServoState_Request_message_members,
  get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_type_hash,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_type_description,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace ros_robot_controller_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Request>()
{
  return &::ros_robot_controller_msgs::srv::rosidl_typesupport_introspection_cpp::GetPWMServoState_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros_robot_controller_msgs, srv, GetPWMServoState_Request)() {
  return &::ros_robot_controller_msgs::srv::rosidl_typesupport_introspection_cpp::GetPWMServoState_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__functions.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros_robot_controller_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetPWMServoState_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros_robot_controller_msgs::srv::GetPWMServoState_Response(_init);
}

void GetPWMServoState_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros_robot_controller_msgs::srv::GetPWMServoState_Response *>(message_memory);
  typed_message->~GetPWMServoState_Response();
}

size_t size_function__GetPWMServoState_Response__state(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ros_robot_controller_msgs::msg::PWMServoState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetPWMServoState_Response__state(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ros_robot_controller_msgs::msg::PWMServoState> *>(untyped_member);
  return &member[index];
}

void * get_function__GetPWMServoState_Response__state(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ros_robot_controller_msgs::msg::PWMServoState> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetPWMServoState_Response__state(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const ros_robot_controller_msgs::msg::PWMServoState *>(
    get_const_function__GetPWMServoState_Response__state(untyped_member, index));
  auto & value = *reinterpret_cast<ros_robot_controller_msgs::msg::PWMServoState *>(untyped_value);
  value = item;
}

void assign_function__GetPWMServoState_Response__state(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<ros_robot_controller_msgs::msg::PWMServoState *>(
    get_function__GetPWMServoState_Response__state(untyped_member, index));
  const auto & value = *reinterpret_cast<const ros_robot_controller_msgs::msg::PWMServoState *>(untyped_value);
  item = value;
}

void resize_function__GetPWMServoState_Response__state(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ros_robot_controller_msgs::msg::PWMServoState> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetPWMServoState_Response_message_member_array[2] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::srv::GetPWMServoState_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros_robot_controller_msgs::msg::PWMServoState>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::srv::GetPWMServoState_Response, state),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetPWMServoState_Response__state,  // size() function pointer
    get_const_function__GetPWMServoState_Response__state,  // get_const(index) function pointer
    get_function__GetPWMServoState_Response__state,  // get(index) function pointer
    fetch_function__GetPWMServoState_Response__state,  // fetch(index, &value) function pointer
    assign_function__GetPWMServoState_Response__state,  // assign(index, value) function pointer
    resize_function__GetPWMServoState_Response__state  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetPWMServoState_Response_message_members = {
  "ros_robot_controller_msgs::srv",  // message namespace
  "GetPWMServoState_Response",  // message name
  2,  // number of fields
  sizeof(ros_robot_controller_msgs::srv::GetPWMServoState_Response),
  false,  // has_any_key_member_
  GetPWMServoState_Response_message_member_array,  // message members
  GetPWMServoState_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GetPWMServoState_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetPWMServoState_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetPWMServoState_Response_message_members,
  get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_type_hash,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_type_description,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace ros_robot_controller_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Response>()
{
  return &::ros_robot_controller_msgs::srv::rosidl_typesupport_introspection_cpp::GetPWMServoState_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros_robot_controller_msgs, srv, GetPWMServoState_Response)() {
  return &::ros_robot_controller_msgs::srv::rosidl_typesupport_introspection_cpp::GetPWMServoState_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__functions.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros_robot_controller_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetPWMServoState_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros_robot_controller_msgs::srv::GetPWMServoState_Event(_init);
}

void GetPWMServoState_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros_robot_controller_msgs::srv::GetPWMServoState_Event *>(message_memory);
  typed_message->~GetPWMServoState_Event();
}

size_t size_function__GetPWMServoState_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ros_robot_controller_msgs::srv::GetPWMServoState_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetPWMServoState_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ros_robot_controller_msgs::srv::GetPWMServoState_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__GetPWMServoState_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ros_robot_controller_msgs::srv::GetPWMServoState_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetPWMServoState_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const ros_robot_controller_msgs::srv::GetPWMServoState_Request *>(
    get_const_function__GetPWMServoState_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<ros_robot_controller_msgs::srv::GetPWMServoState_Request *>(untyped_value);
  value = item;
}

void assign_function__GetPWMServoState_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<ros_robot_controller_msgs::srv::GetPWMServoState_Request *>(
    get_function__GetPWMServoState_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const ros_robot_controller_msgs::srv::GetPWMServoState_Request *>(untyped_value);
  item = value;
}

void resize_function__GetPWMServoState_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ros_robot_controller_msgs::srv::GetPWMServoState_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetPWMServoState_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ros_robot_controller_msgs::srv::GetPWMServoState_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetPWMServoState_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ros_robot_controller_msgs::srv::GetPWMServoState_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__GetPWMServoState_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ros_robot_controller_msgs::srv::GetPWMServoState_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetPWMServoState_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const ros_robot_controller_msgs::srv::GetPWMServoState_Response *>(
    get_const_function__GetPWMServoState_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<ros_robot_controller_msgs::srv::GetPWMServoState_Response *>(untyped_value);
  value = item;
}

void assign_function__GetPWMServoState_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<ros_robot_controller_msgs::srv::GetPWMServoState_Response *>(
    get_function__GetPWMServoState_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const ros_robot_controller_msgs::srv::GetPWMServoState_Response *>(untyped_value);
  item = value;
}

void resize_function__GetPWMServoState_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ros_robot_controller_msgs::srv::GetPWMServoState_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetPWMServoState_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::srv::GetPWMServoState_Event, info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "request",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(ros_robot_controller_msgs::srv::GetPWMServoState_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetPWMServoState_Event__request,  // size() function pointer
    get_const_function__GetPWMServoState_Event__request,  // get_const(index) function pointer
    get_function__GetPWMServoState_Event__request,  // get(index) function pointer
    fetch_function__GetPWMServoState_Event__request,  // fetch(index, &value) function pointer
    assign_function__GetPWMServoState_Event__request,  // assign(index, value) function pointer
    resize_function__GetPWMServoState_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(ros_robot_controller_msgs::srv::GetPWMServoState_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetPWMServoState_Event__response,  // size() function pointer
    get_const_function__GetPWMServoState_Event__response,  // get_const(index) function pointer
    get_function__GetPWMServoState_Event__response,  // get(index) function pointer
    fetch_function__GetPWMServoState_Event__response,  // fetch(index, &value) function pointer
    assign_function__GetPWMServoState_Event__response,  // assign(index, value) function pointer
    resize_function__GetPWMServoState_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetPWMServoState_Event_message_members = {
  "ros_robot_controller_msgs::srv",  // message namespace
  "GetPWMServoState_Event",  // message name
  3,  // number of fields
  sizeof(ros_robot_controller_msgs::srv::GetPWMServoState_Event),
  false,  // has_any_key_member_
  GetPWMServoState_Event_message_member_array,  // message members
  GetPWMServoState_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  GetPWMServoState_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetPWMServoState_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetPWMServoState_Event_message_members,
  get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_type_hash,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_type_description,
  &ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace ros_robot_controller_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Event>()
{
  return &::ros_robot_controller_msgs::srv::rosidl_typesupport_introspection_cpp::GetPWMServoState_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros_robot_controller_msgs, srv, GetPWMServoState_Event)() {
  return &::ros_robot_controller_msgs::srv::rosidl_typesupport_introspection_cpp::GetPWMServoState_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__functions.h"
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace ros_robot_controller_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers GetPWMServoState_service_members = {
  "ros_robot_controller_msgs::srv",  // service namespace
  "GetPWMServoState",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t GetPWMServoState_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetPWMServoState_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<ros_robot_controller_msgs::srv::GetPWMServoState>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<ros_robot_controller_msgs::srv::GetPWMServoState>,
  &ros_robot_controller_msgs__srv__GetPWMServoState__get_type_hash,
  &ros_robot_controller_msgs__srv__GetPWMServoState__get_type_description,
  &ros_robot_controller_msgs__srv__GetPWMServoState__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace ros_robot_controller_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::ros_robot_controller_msgs::srv::rosidl_typesupport_introspection_cpp::GetPWMServoState_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure all of the service_members are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr ||
    service_members->event_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::ros_robot_controller_msgs::srv::GetPWMServoState_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::ros_robot_controller_msgs::srv::GetPWMServoState_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::ros_robot_controller_msgs::srv::GetPWMServoState_Event
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros_robot_controller_msgs, srv, GetPWMServoState)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<ros_robot_controller_msgs::srv::GetPWMServoState>();
}

#ifdef __cplusplus
}
#endif
