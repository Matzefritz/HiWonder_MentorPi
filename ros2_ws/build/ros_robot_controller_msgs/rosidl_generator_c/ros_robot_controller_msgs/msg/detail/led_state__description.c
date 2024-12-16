// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:msg/LedState.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/msg/detail/led_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__LedState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xcf, 0xf5, 0xa2, 0xe1, 0x6f, 0xe1, 0xee, 0x74,
      0xb9, 0x68, 0x40, 0x67, 0xcd, 0xd0, 0x0e, 0x74,
      0x75, 0x3d, 0x10, 0x98, 0x8b, 0x76, 0xe1, 0xf0,
      0xc0, 0x48, 0x9e, 0x61, 0x73, 0xdc, 0x8b, 0xd3,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ros_robot_controller_msgs__msg__LedState__TYPE_NAME[] = "ros_robot_controller_msgs/msg/LedState";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__msg__LedState__FIELD_NAME__id[] = "id";
static char ros_robot_controller_msgs__msg__LedState__FIELD_NAME__on_time[] = "on_time";
static char ros_robot_controller_msgs__msg__LedState__FIELD_NAME__off_time[] = "off_time";
static char ros_robot_controller_msgs__msg__LedState__FIELD_NAME__repeat[] = "repeat";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__msg__LedState__FIELDS[] = {
  {
    {ros_robot_controller_msgs__msg__LedState__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__LedState__FIELD_NAME__on_time, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__LedState__FIELD_NAME__off_time, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__LedState__FIELD_NAME__repeat, 6, 6},
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
ros_robot_controller_msgs__msg__LedState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__msg__LedState__TYPE_NAME, 38, 38},
      {ros_robot_controller_msgs__msg__LedState__FIELDS, 4, 4},
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
  "float32 on_time\n"
  "float32 off_time\n"
  "uint16 repeat";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__LedState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__msg__LedState__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 56, 56},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__LedState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__msg__LedState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
