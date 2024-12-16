// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros_robot_controller_msgs:msg/PWMServoState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros_robot_controller_msgs/msg/detail/pwm_servo_state__rosidl_typesupport_introspection_c.h"
#include "ros_robot_controller_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros_robot_controller_msgs/msg/detail/pwm_servo_state__functions.h"
#include "ros_robot_controller_msgs/msg/detail/pwm_servo_state__struct.h"


// Include directives for member types
// Member `id`
// Member `position`
// Member `offset`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__PWMServoState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros_robot_controller_msgs__msg__PWMServoState__init(message_memory);
}

void ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__PWMServoState_fini_function(void * message_memory)
{
  ros_robot_controller_msgs__msg__PWMServoState__fini(message_memory);
}

size_t ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__size_function__PWMServoState__id(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_const_function__PWMServoState__id(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_function__PWMServoState__id(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__fetch_function__PWMServoState__id(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_const_function__PWMServoState__id(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__assign_function__PWMServoState__id(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_function__PWMServoState__id(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__resize_function__PWMServoState__id(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__size_function__PWMServoState__position(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_const_function__PWMServoState__position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_function__PWMServoState__position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__fetch_function__PWMServoState__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_const_function__PWMServoState__position(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__assign_function__PWMServoState__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_function__PWMServoState__position(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__resize_function__PWMServoState__position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__size_function__PWMServoState__offset(
  const void * untyped_member)
{
  const rosidl_runtime_c__int16__Sequence * member =
    (const rosidl_runtime_c__int16__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_const_function__PWMServoState__offset(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int16__Sequence * member =
    (const rosidl_runtime_c__int16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_function__PWMServoState__offset(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int16__Sequence * member =
    (rosidl_runtime_c__int16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__fetch_function__PWMServoState__offset(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_const_function__PWMServoState__offset(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__assign_function__PWMServoState__offset(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_function__PWMServoState__offset(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

bool ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__resize_function__PWMServoState__offset(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int16__Sequence * member =
    (rosidl_runtime_c__int16__Sequence *)(untyped_member);
  rosidl_runtime_c__int16__Sequence__fini(member);
  return rosidl_runtime_c__int16__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__PWMServoState_message_member_array[3] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_robot_controller_msgs__msg__PWMServoState, id),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__size_function__PWMServoState__id,  // size() function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_const_function__PWMServoState__id,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_function__PWMServoState__id,  // get(index) function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__fetch_function__PWMServoState__id,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__assign_function__PWMServoState__id,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__resize_function__PWMServoState__id  // resize(index) function pointer
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
    offsetof(ros_robot_controller_msgs__msg__PWMServoState, position),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__size_function__PWMServoState__position,  // size() function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_const_function__PWMServoState__position,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_function__PWMServoState__position,  // get(index) function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__fetch_function__PWMServoState__position,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__assign_function__PWMServoState__position,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__resize_function__PWMServoState__position  // resize(index) function pointer
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
    offsetof(ros_robot_controller_msgs__msg__PWMServoState, offset),  // bytes offset in struct
    NULL,  // default value
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__size_function__PWMServoState__offset,  // size() function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_const_function__PWMServoState__offset,  // get_const(index) function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__get_function__PWMServoState__offset,  // get(index) function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__fetch_function__PWMServoState__offset,  // fetch(index, &value) function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__assign_function__PWMServoState__offset,  // assign(index, value) function pointer
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__resize_function__PWMServoState__offset  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__PWMServoState_message_members = {
  "ros_robot_controller_msgs__msg",  // message namespace
  "PWMServoState",  // message name
  3,  // number of fields
  sizeof(ros_robot_controller_msgs__msg__PWMServoState),
  false,  // has_any_key_member_
  ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__PWMServoState_message_member_array,  // message members
  ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__PWMServoState_init_function,  // function to initialize message memory (memory has to be allocated)
  ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__PWMServoState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__PWMServoState_message_type_support_handle = {
  0,
  &ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__PWMServoState_message_members,
  get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__msg__PWMServoState__get_type_hash,
  &ros_robot_controller_msgs__msg__PWMServoState__get_type_description,
  &ros_robot_controller_msgs__msg__PWMServoState__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros_robot_controller_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_robot_controller_msgs, msg, PWMServoState)() {
  if (!ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__PWMServoState_message_type_support_handle.typesupport_identifier) {
    ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__PWMServoState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros_robot_controller_msgs__msg__PWMServoState__rosidl_typesupport_introspection_c__PWMServoState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
