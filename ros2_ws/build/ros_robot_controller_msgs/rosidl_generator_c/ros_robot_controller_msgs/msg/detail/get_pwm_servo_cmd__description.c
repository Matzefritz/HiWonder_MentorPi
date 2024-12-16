// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:msg/GetPWMServoCmd.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/msg/detail/get_pwm_servo_cmd__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__GetPWMServoCmd__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe3, 0xe0, 0xa0, 0x6e, 0xd3, 0xf4, 0x9c, 0x40,
      0x0d, 0x6e, 0x24, 0xfa, 0x5a, 0x9e, 0xc3, 0x8a,
      0xe8, 0x0d, 0xe3, 0x64, 0x27, 0x87, 0xcb, 0x3c,
      0xaa, 0x06, 0x10, 0x8c, 0xbc, 0x93, 0xf4, 0xc4,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ros_robot_controller_msgs__msg__GetPWMServoCmd__TYPE_NAME[] = "ros_robot_controller_msgs/msg/GetPWMServoCmd";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__msg__GetPWMServoCmd__FIELD_NAME__id[] = "id";
static char ros_robot_controller_msgs__msg__GetPWMServoCmd__FIELD_NAME__get_position[] = "get_position";
static char ros_robot_controller_msgs__msg__GetPWMServoCmd__FIELD_NAME__get_offset[] = "get_offset";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__msg__GetPWMServoCmd__FIELDS[] = {
  {
    {ros_robot_controller_msgs__msg__GetPWMServoCmd__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__GetPWMServoCmd__FIELD_NAME__get_position, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__GetPWMServoCmd__FIELD_NAME__get_offset, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros_robot_controller_msgs__msg__GetPWMServoCmd__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__msg__GetPWMServoCmd__TYPE_NAME, 44, 44},
      {ros_robot_controller_msgs__msg__GetPWMServoCmd__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 id\n"
  "uint8 get_position\n"
  "uint8 get_offset";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__GetPWMServoCmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__msg__GetPWMServoCmd__TYPE_NAME, 44, 44},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 45, 45},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__GetPWMServoCmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__msg__GetPWMServoCmd__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
