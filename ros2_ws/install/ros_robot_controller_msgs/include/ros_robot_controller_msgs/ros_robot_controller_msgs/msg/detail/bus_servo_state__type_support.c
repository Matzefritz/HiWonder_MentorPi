// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros_robot_controller_msgs:msg/BusServoState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros_robot_controller_msgs/msg/detail/bus_servo_state__rosidl_typesupport_introspection_c.h"
#include "ros_robot_controller_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros_robot_controller_msgs/msg/detail/bus_servo_state__functions.h"
#include "ros_robot_controller_msgs/msg/detail/bus_servo_state__struct.h"


// Include directives for member types
// Member `present_id`
// Member `target_id`
// Member `position`
// Member `offset`
// Member `voltage`
// Member `temperature`
// Member `position_limit`
// Member `voltage_limit`
// Member `max_temperature_limit`
// Member `enable_torque`
// Member `save_offset`
// Member `stop`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__BusServoState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros_robot_controller_msgs__msg__BusServoState__init(message_memory);
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__BusServoState_fini_function(void * message_memory)
{
  ros_robot_controller_msgs__msg__BusServoState__fini(message_memory);
}

size_t ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__present_id(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__present_id(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__present_id(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__present_id(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__present_id(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__present_id(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__present_id(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__present_id(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__target_id(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__target_id(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__target_id(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__target_id(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__target_id(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__target_id(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__target_id(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__target_id(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__position(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__position(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__position(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__offset(
  const void * untyped_member)
{
  const rosidl_runtime_c__int16__Sequence * member =
    (const rosidl_runtime_c__int16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__offset(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int16__Sequence * member =
    (const rosidl_runtime_c__int16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__offset(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int16__Sequence * member =
    (rosidl_runtime_c__int16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__offset(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__offset(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__offset(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__offset(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__offset(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int16__Sequence * member =
    (rosidl_runtime_c__int16__Sequence *)(untyped_member);
  rosidl_runtime_c__int16__Sequence__fini(member);
  return rosidl_runtime_c__int16__Sequence__init(member, size);
}

size_t ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__voltage(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__voltage(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__voltage(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__voltage(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__voltage(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__voltage(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__voltage(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__voltage(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__temperature(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__temperature(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__temperature(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__temperature(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__temperature(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__temperature(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__temperature(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__temperature(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__position_limit(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__position_limit(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__position_limit(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__position_limit(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__position_limit(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__position_limit(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__position_limit(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__position_limit(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__voltage_limit(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__voltage_limit(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__voltage_limit(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__voltage_limit(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__voltage_limit(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__voltage_limit(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__voltage_limit(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__voltage_limit(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__max_temperature_limit(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__max_temperature_limit(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__max_temperature_limit(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__max_temperature_limit(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__max_temperature_limit(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__max_temperature_limit(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__max_temperature_limit(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__max_temperature_limit(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__enable_torque(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__enable_torque(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__enable_torque(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__enable_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__enable_torque(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__enable_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__enable_torque(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__enable_torque(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__save_offset(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__save_offset(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__save_offset(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__save_offset(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__save_offset(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__save_offset(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__save_offset(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__save_offset(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__stop(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__stop(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__stop(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__stop(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__stop(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__stop(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__stop(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__stop(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__BusServoState_message_member_array[12] = {
  {
    "present_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs__msg__BusServoState, present_id),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__present_id,  // size() function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__present_id,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__present_id,  // get(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__present_id,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__present_id,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__present_id  // resize(index) function pointer
  },
  {
    "target_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs__msg__BusServoState, target_id),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__target_id,  // size() function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__target_id,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__target_id,  // get(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__target_id,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__target_id,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__target_id  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs__msg__BusServoState, position),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__position,  // size() function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__position,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__position,  // get(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__position,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__position,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__position  // resize(index) function pointer
  },
  {
    "offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs__msg__BusServoState, offset),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__offset,  // size() function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__offset,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__offset,  // get(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__offset,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__offset,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__offset  // resize(index) function pointer
  },
  {
    "voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs__msg__BusServoState, voltage),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__voltage,  // size() function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__voltage,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__voltage,  // get(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__voltage,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__voltage,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__voltage  // resize(index) function pointer
  },
  {
    "temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs__msg__BusServoState, temperature),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__temperature,  // size() function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__temperature,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__temperature,  // get(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__temperature,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__temperature,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__temperature  // resize(index) function pointer
  },
  {
    "position_limit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs__msg__BusServoState, position_limit),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__position_limit,  // size() function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__position_limit,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__position_limit,  // get(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__position_limit,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__position_limit,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__position_limit  // resize(index) function pointer
  },
  {
    "voltage_limit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs__msg__BusServoState, voltage_limit),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__voltage_limit,  // size() function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__voltage_limit,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__voltage_limit,  // get(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__voltage_limit,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__voltage_limit,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__voltage_limit  // resize(index) function pointer
  },
  {
    "max_temperature_limit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs__msg__BusServoState, max_temperature_limit),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__max_temperature_limit,  // size() function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__max_temperature_limit,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__max_temperature_limit,  // get(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__max_temperature_limit,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__max_temperature_limit,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__max_temperature_limit  // resize(index) function pointer
  },
  {
    "enable_torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs__msg__BusServoState, enable_torque),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__enable_torque,  // size() function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__enable_torque,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__enable_torque,  // get(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__enable_torque,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__enable_torque,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__enable_torque  // resize(index) function pointer
  },
  {
    "save_offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs__msg__BusServoState, save_offset),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__save_offset,  // size() function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__save_offset,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__save_offset,  // get(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__save_offset,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__save_offset,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__save_offset  // resize(index) function pointer
  },
  {
    "stop",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs__msg__BusServoState, stop),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__size_function__BusServoState__stop,  // size() function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_const_function__BusServoState__stop,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__get_function__BusServoState__stop,  // get(index) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__fetch_function__BusServoState__stop,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__assign_function__BusServoState__stop,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__resize_function__BusServoState__stop  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__BusServoState_message_members = {
  "ros_robot_controller_msgs__msg",  // message namespace
  "BusServoState",  // message name
  12,  // number of fields
  sizeof(ros_robot_controller_msgs__msg__BusServoState),
  false,  // has_any_key_member_
  ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__BusServoState_message_member_array,  // message members
  ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__BusServoState_init_function,  // function to initialize message memory (memory has to be allocated)
  ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__BusServoState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__BusServoState_message_type_support_handle = {
  0,
  &ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__BusServoState_message_members,
  get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__msg__BusServoState__get_type_hash,
  &ros_robot_controller_msgs__msg__BusServoState__get_type_description,
  &ros_robot_controller_msgs__msg__BusServoState__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros_robot_controller_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_robot_controller_msgs, msg, BusServoState)() {
  if (!ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__BusServoState_message_type_support_handle.typesupport_identifier) {
    ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__BusServoState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros_robot_controller_msgs__msg__BusServoState__rosidl_typesupport_introspection_c__BusServoState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
