// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ros_robot_controller_msgs:msg/SetPWMServoState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ros_robot_controller_msgs/msg/detail/set_pwm_servo_state__functions.h"
#include "ros_robot_controller_msgs/msg/detail/set_pwm_servo_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros_robot_controller_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SetPWMServoState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros_robot_controller_msgs::msg::SetPWMServoState(_init);
}

void SetPWMServoState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros_robot_controller_msgs::msg::SetPWMServoState *>(message_memory);
  typed_message->~SetPWMServoState();
}

size_t size_function__SetPWMServoState__state(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ros_robot_controller_msgs::msg::PWMServoState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SetPWMServoState__state(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ros_robot_controller_msgs::msg::PWMServoState> *>(untyped_member);
  return &member[index];
}

void * get_function__SetPWMServoState__state(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ros_robot_controller_msgs::msg::PWMServoState> *>(untyped_member);
  return &member[index];
}

void fetch_function__SetPWMServoState__state(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const ros_robot_controller_msgs::msg::PWMServoState *>(
    get_const_function__SetPWMServoState__state(untyped_member, index));
  auto & value = *reinterpret_cast<ros_robot_controller_msgs::msg::PWMServoState *>(untyped_value);
  value = item;
}

void assign_function__SetPWMServoState__state(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<ros_robot_controller_msgs::msg::PWMServoState *>(
    get_function__SetPWMServoState__state(untyped_member, index));
  const auto & value = *reinterpret_cast<const ros_robot_controller_msgs::msg::PWMServoState *>(untyped_value);
  item = value;
}

void resize_function__SetPWMServoState__state(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ros_robot_controller_msgs::msg::PWMServoState> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetPWMServoState_message_member_array[2] = {
  {
    "state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros_robot_controller_msgs::msg::PWMServoState>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::SetPWMServoState, state),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetPWMServoState__state,  // size() function pointer
    get_const_function__SetPWMServoState__state,  // get_const(index) function pointer
    get_function__SetPWMServoState__state,  // get(index) function pointer
    fetch_function__SetPWMServoState__state,  // fetch(index, &value) function pointer
    assign_function__SetPWMServoState__state,  // assign(index, value) function pointer
    resize_function__SetPWMServoState__state  // resize(index) function pointer
  },
  {
    "duration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::SetPWMServoState, duration),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetPWMServoState_message_members = {
  "ros_robot_controller_msgs::msg",  // message namespace
  "SetPWMServoState",  // message name
  2,  // number of fields
  sizeof(ros_robot_controller_msgs::msg::SetPWMServoState),
  false,  // has_any_key_member_
  SetPWMServoState_message_member_array,  // message members
  SetPWMServoState_init_function,  // function to initialize message memory (memory has to be allocated)
  SetPWMServoState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetPWMServoState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetPWMServoState_message_members,
  get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__msg__SetPWMServoState__get_type_hash,
  &ros_robot_controller_msgs__msg__SetPWMServoState__get_type_description,
  &ros_robot_controller_msgs__msg__SetPWMServoState__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ros_robot_controller_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_robot_controller_msgs::msg::SetPWMServoState>()
{
  return &::ros_robot_controller_msgs::msg::rosidl_typesupport_introspection_cpp::SetPWMServoState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros_robot_controller_msgs, msg, SetPWMServoState)() {
  return &::ros_robot_controller_msgs::msg::rosidl_typesupport_introspection_cpp::SetPWMServoState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
