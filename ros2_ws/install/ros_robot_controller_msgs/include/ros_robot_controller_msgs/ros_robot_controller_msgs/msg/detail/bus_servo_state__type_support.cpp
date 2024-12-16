// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ros_robot_controller_msgs:msg/BusServoState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ros_robot_controller_msgs/msg/detail/bus_servo_state__functions.h"
#include "ros_robot_controller_msgs/msg/detail/bus_servo_state__struct.hpp"
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

void BusServoState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros_robot_controller_msgs::msg::BusServoState(_init);
}

void BusServoState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros_robot_controller_msgs::msg::BusServoState *>(message_memory);
  typed_message->~BusServoState();
}

size_t size_function__BusServoState__present_id(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BusServoState__present_id(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__BusServoState__present_id(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__BusServoState__present_id(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__BusServoState__present_id(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__BusServoState__present_id(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__BusServoState__present_id(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__BusServoState__present_id(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BusServoState__target_id(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BusServoState__target_id(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__BusServoState__target_id(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__BusServoState__target_id(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__BusServoState__target_id(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__BusServoState__target_id(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__BusServoState__target_id(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__BusServoState__target_id(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BusServoState__position(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BusServoState__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__BusServoState__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__BusServoState__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__BusServoState__position(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__BusServoState__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__BusServoState__position(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__BusServoState__position(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BusServoState__offset(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BusServoState__offset(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__BusServoState__offset(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__BusServoState__offset(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int16_t *>(
    get_const_function__BusServoState__offset(untyped_member, index));
  auto & value = *reinterpret_cast<int16_t *>(untyped_value);
  value = item;
}

void assign_function__BusServoState__offset(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int16_t *>(
    get_function__BusServoState__offset(untyped_member, index));
  const auto & value = *reinterpret_cast<const int16_t *>(untyped_value);
  item = value;
}

void resize_function__BusServoState__offset(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int16_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BusServoState__voltage(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BusServoState__voltage(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__BusServoState__voltage(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__BusServoState__voltage(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__BusServoState__voltage(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__BusServoState__voltage(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__BusServoState__voltage(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__BusServoState__voltage(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BusServoState__temperature(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BusServoState__temperature(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__BusServoState__temperature(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__BusServoState__temperature(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__BusServoState__temperature(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__BusServoState__temperature(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__BusServoState__temperature(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__BusServoState__temperature(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BusServoState__position_limit(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BusServoState__position_limit(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__BusServoState__position_limit(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__BusServoState__position_limit(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__BusServoState__position_limit(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__BusServoState__position_limit(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__BusServoState__position_limit(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__BusServoState__position_limit(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BusServoState__voltage_limit(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BusServoState__voltage_limit(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__BusServoState__voltage_limit(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__BusServoState__voltage_limit(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__BusServoState__voltage_limit(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__BusServoState__voltage_limit(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__BusServoState__voltage_limit(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__BusServoState__voltage_limit(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BusServoState__max_temperature_limit(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BusServoState__max_temperature_limit(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__BusServoState__max_temperature_limit(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__BusServoState__max_temperature_limit(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__BusServoState__max_temperature_limit(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__BusServoState__max_temperature_limit(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__BusServoState__max_temperature_limit(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__BusServoState__max_temperature_limit(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BusServoState__enable_torque(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BusServoState__enable_torque(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__BusServoState__enable_torque(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__BusServoState__enable_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__BusServoState__enable_torque(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__BusServoState__enable_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__BusServoState__enable_torque(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__BusServoState__enable_torque(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BusServoState__save_offset(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BusServoState__save_offset(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__BusServoState__save_offset(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__BusServoState__save_offset(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__BusServoState__save_offset(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__BusServoState__save_offset(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__BusServoState__save_offset(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__BusServoState__save_offset(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BusServoState__stop(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BusServoState__stop(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__BusServoState__stop(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__BusServoState__stop(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__BusServoState__stop(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__BusServoState__stop(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__BusServoState__stop(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__BusServoState__stop(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BusServoState_message_member_array[12] = {
  {
    "present_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::BusServoState, present_id),  // bytes offset in struct
    nullptr,  // default value
    size_function__BusServoState__present_id,  // size() function pointer
    get_const_function__BusServoState__present_id,  // get_const(index) function pointer
    get_function__BusServoState__present_id,  // get(index) function pointer
    fetch_function__BusServoState__present_id,  // fetch(index, &value) function pointer
    assign_function__BusServoState__present_id,  // assign(index, value) function pointer
    resize_function__BusServoState__present_id  // resize(index) function pointer
  },
  {
    "target_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::BusServoState, target_id),  // bytes offset in struct
    nullptr,  // default value
    size_function__BusServoState__target_id,  // size() function pointer
    get_const_function__BusServoState__target_id,  // get_const(index) function pointer
    get_function__BusServoState__target_id,  // get(index) function pointer
    fetch_function__BusServoState__target_id,  // fetch(index, &value) function pointer
    assign_function__BusServoState__target_id,  // assign(index, value) function pointer
    resize_function__BusServoState__target_id  // resize(index) function pointer
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::BusServoState, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__BusServoState__position,  // size() function pointer
    get_const_function__BusServoState__position,  // get_const(index) function pointer
    get_function__BusServoState__position,  // get(index) function pointer
    fetch_function__BusServoState__position,  // fetch(index, &value) function pointer
    assign_function__BusServoState__position,  // assign(index, value) function pointer
    resize_function__BusServoState__position  // resize(index) function pointer
  },
  {
    "offset",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::BusServoState, offset),  // bytes offset in struct
    nullptr,  // default value
    size_function__BusServoState__offset,  // size() function pointer
    get_const_function__BusServoState__offset,  // get_const(index) function pointer
    get_function__BusServoState__offset,  // get(index) function pointer
    fetch_function__BusServoState__offset,  // fetch(index, &value) function pointer
    assign_function__BusServoState__offset,  // assign(index, value) function pointer
    resize_function__BusServoState__offset  // resize(index) function pointer
  },
  {
    "voltage",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::BusServoState, voltage),  // bytes offset in struct
    nullptr,  // default value
    size_function__BusServoState__voltage,  // size() function pointer
    get_const_function__BusServoState__voltage,  // get_const(index) function pointer
    get_function__BusServoState__voltage,  // get(index) function pointer
    fetch_function__BusServoState__voltage,  // fetch(index, &value) function pointer
    assign_function__BusServoState__voltage,  // assign(index, value) function pointer
    resize_function__BusServoState__voltage  // resize(index) function pointer
  },
  {
    "temperature",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::BusServoState, temperature),  // bytes offset in struct
    nullptr,  // default value
    size_function__BusServoState__temperature,  // size() function pointer
    get_const_function__BusServoState__temperature,  // get_const(index) function pointer
    get_function__BusServoState__temperature,  // get(index) function pointer
    fetch_function__BusServoState__temperature,  // fetch(index, &value) function pointer
    assign_function__BusServoState__temperature,  // assign(index, value) function pointer
    resize_function__BusServoState__temperature  // resize(index) function pointer
  },
  {
    "position_limit",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::BusServoState, position_limit),  // bytes offset in struct
    nullptr,  // default value
    size_function__BusServoState__position_limit,  // size() function pointer
    get_const_function__BusServoState__position_limit,  // get_const(index) function pointer
    get_function__BusServoState__position_limit,  // get(index) function pointer
    fetch_function__BusServoState__position_limit,  // fetch(index, &value) function pointer
    assign_function__BusServoState__position_limit,  // assign(index, value) function pointer
    resize_function__BusServoState__position_limit  // resize(index) function pointer
  },
  {
    "voltage_limit",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::BusServoState, voltage_limit),  // bytes offset in struct
    nullptr,  // default value
    size_function__BusServoState__voltage_limit,  // size() function pointer
    get_const_function__BusServoState__voltage_limit,  // get_const(index) function pointer
    get_function__BusServoState__voltage_limit,  // get(index) function pointer
    fetch_function__BusServoState__voltage_limit,  // fetch(index, &value) function pointer
    assign_function__BusServoState__voltage_limit,  // assign(index, value) function pointer
    resize_function__BusServoState__voltage_limit  // resize(index) function pointer
  },
  {
    "max_temperature_limit",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::BusServoState, max_temperature_limit),  // bytes offset in struct
    nullptr,  // default value
    size_function__BusServoState__max_temperature_limit,  // size() function pointer
    get_const_function__BusServoState__max_temperature_limit,  // get_const(index) function pointer
    get_function__BusServoState__max_temperature_limit,  // get(index) function pointer
    fetch_function__BusServoState__max_temperature_limit,  // fetch(index, &value) function pointer
    assign_function__BusServoState__max_temperature_limit,  // assign(index, value) function pointer
    resize_function__BusServoState__max_temperature_limit  // resize(index) function pointer
  },
  {
    "enable_torque",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::BusServoState, enable_torque),  // bytes offset in struct
    nullptr,  // default value
    size_function__BusServoState__enable_torque,  // size() function pointer
    get_const_function__BusServoState__enable_torque,  // get_const(index) function pointer
    get_function__BusServoState__enable_torque,  // get(index) function pointer
    fetch_function__BusServoState__enable_torque,  // fetch(index, &value) function pointer
    assign_function__BusServoState__enable_torque,  // assign(index, value) function pointer
    resize_function__BusServoState__enable_torque  // resize(index) function pointer
  },
  {
    "save_offset",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::BusServoState, save_offset),  // bytes offset in struct
    nullptr,  // default value
    size_function__BusServoState__save_offset,  // size() function pointer
    get_const_function__BusServoState__save_offset,  // get_const(index) function pointer
    get_function__BusServoState__save_offset,  // get(index) function pointer
    fetch_function__BusServoState__save_offset,  // fetch(index, &value) function pointer
    assign_function__BusServoState__save_offset,  // assign(index, value) function pointer
    resize_function__BusServoState__save_offset  // resize(index) function pointer
  },
  {
    "stop",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs::msg::BusServoState, stop),  // bytes offset in struct
    nullptr,  // default value
    size_function__BusServoState__stop,  // size() function pointer
    get_const_function__BusServoState__stop,  // get_const(index) function pointer
    get_function__BusServoState__stop,  // get(index) function pointer
    fetch_function__BusServoState__stop,  // fetch(index, &value) function pointer
    assign_function__BusServoState__stop,  // assign(index, value) function pointer
    resize_function__BusServoState__stop  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BusServoState_message_members = {
  "ros_robot_controller_msgs::msg",  // message namespace
  "BusServoState",  // message name
  12,  // number of fields
  sizeof(ros_robot_controller_msgs::msg::BusServoState),
  false,  // has_any_key_member_
  BusServoState_message_member_array,  // message members
  BusServoState_init_function,  // function to initialize message memory (memory has to be allocated)
  BusServoState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BusServoState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BusServoState_message_members,
  get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__msg__BusServoState__get_type_hash,
  &ros_robot_controller_msgs__msg__BusServoState__get_type_description,
  &ros_robot_controller_msgs__msg__BusServoState__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ros_robot_controller_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_robot_controller_msgs::msg::BusServoState>()
{
  return &::ros_robot_controller_msgs::msg::rosidl_typesupport_introspection_cpp::BusServoState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros_robot_controller_msgs, msg, BusServoState)() {
  return &::ros_robot_controller_msgs::msg::rosidl_typesupport_introspection_cpp::BusServoState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
