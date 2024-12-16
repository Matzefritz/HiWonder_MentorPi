// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:msg/BuzzerState.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/msg/detail/buzzer_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__BuzzerState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x55, 0x97, 0x44, 0x4e, 0xb2, 0x21, 0x9d, 0xba,
      0xd6, 0x55, 0xd4, 0xa9, 0xf5, 0x21, 0xd0, 0xf1,
      0x0e, 0x46, 0xe7, 0x36, 0xd8, 0xa1, 0x3e, 0xf1,
      0xae, 0xcd, 0xfe, 0x6a, 0x9a, 0xf0, 0x66, 0x2c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ros_robot_controller_msgs__msg__BuzzerState__TYPE_NAME[] = "ros_robot_controller_msgs/msg/BuzzerState";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__msg__BuzzerState__FIELD_NAME__freq[] = "freq";
static char ros_robot_controller_msgs__msg__BuzzerState__FIELD_NAME__on_time[] = "on_time";
static char ros_robot_controller_msgs__msg__BuzzerState__FIELD_NAME__off_time[] = "off_time";
static char ros_robot_controller_msgs__msg__BuzzerState__FIELD_NAME__repeat[] = "repeat";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__msg__BuzzerState__FIELDS[] = {
  {
    {ros_robot_controller_msgs__msg__BuzzerState__FIELD_NAME__freq, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__BuzzerState__FIELD_NAME__on_time, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__BuzzerState__FIELD_NAME__off_time, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__BuzzerState__FIELD_NAME__repeat, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros_robot_controller_msgs__msg__BuzzerState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__msg__BuzzerState__TYPE_NAME, 41, 41},
      {ros_robot_controller_msgs__msg__BuzzerState__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint16 freq\n"
  "float32 on_time\n"
  "float32 off_time\n"
  "uint16 repeat";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__BuzzerState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__msg__BuzzerState__TYPE_NAME, 41, 41},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 59, 59},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__BuzzerState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__msg__BuzzerState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
