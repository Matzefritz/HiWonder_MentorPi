// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ros_robot_controller_msgs:msg/ServosPosition.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ros_robot_controller_msgs/msg/detail/servos_position__functions.h"
#include "ros_robot_controller_msgs/msg/detail/servos_position__struct.hpp"
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

void ServosPosition_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros_robot_controller_msgs::msg::ServosPosition(_init);
}

void ServosPosition_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros_robot_controller_msgs::msg::ServosPosition *>(message_memory);
  typed_message->~ServosPosition();
}

size_t size_function__ServosPosition__position(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ros_robot_controller_msgs::msg::ServoPosition> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ServosPosition__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ros_robot_controller_msgs::msg::ServoPosition> *>(untyped_member);
  return &member[index];
}

void * get_function__ServosPosition__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ros_robot_controller_msgs::msg::ServoPosition> *>(untyped_member);
  return &member[index];
}

void fetch_function__ServosPosition__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const ros_robot_controller_msgs::msg::ServoPosition *>(
    get_const_function__ServosPosition__position(untyped_member, index));
  auto & value = *reinterpret_cast<ros_robot_controller_msgs::msg::ServoPosition *>(untyped_value);
  value = item;
}

void assign_function__ServosPosition__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<ros_robot_controller_msgs::msg::ServoPosition *>(
    get_function__ServosPosition__position(untyped_member, index));
  const auto & value = *reinterpret_cast<const ros_robot_controller_msgs::msg::ServoPosition *>(untyped_value);
  item = value;
}

void resize_function__ServosPosition__position(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ros_robot_controller_msgs::msg::ServoPosition> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ServosPosition_message_member_array[2] = {
  {
    "duration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::ServosPosition, duration),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros_robot_controller_msgs::msg::ServoPosition>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::ServosPosition, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__ServosPosition__position,  // size() function pointer
    get_const_function__ServosPosition__position,  // get_const(index) function pointer
    get_function__ServosPosition__position,  // get(index) function pointer
    fetch_function__ServosPosition__position,  // fetch(index, &value) function pointer
    assign_function__ServosPosition__position,  // assign(index, value) function pointer
    resize_function__ServosPosition__position  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ServosPosition_message_members = {
  "ros_robot_controller_msgs::msg",  // message namespace
  "ServosPosition",  // message name
  2,  // number of fields
  sizeof(ros_robot_controller_msgs::msg::ServosPosition),
  false,  // has_any_key_member_
  ServosPosition_message_member_array,  // message members
  ServosPosition_init_function,  // function to initialize message memory (memory has to be allocated)
  ServosPosition_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ServosPosition_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ServosPosition_message_members,
  get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__msg__ServosPosition__get_type_hash,
  &ros_robot_controller_msgs__msg__ServosPosition__get_type_description,
  &ros_robot_controller_msgs__msg__ServosPosition__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ros_robot_controller_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_robot_controller_msgs::msg::ServosPosition>()
{
  return &::ros_robot_controller_msgs::msg::rosidl_typesupport_introspection_cpp::ServosPosition_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros_robot_controller_msgs, msg, ServosPosition)() {
  return &::ros_robot_controller_msgs::msg::rosidl_typesupport_introspection_cpp::ServosPosition_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
