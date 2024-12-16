// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:msg/RGBState.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/msg/detail/rgb_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__RGBState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xde, 0x85, 0xae, 0x06, 0x06, 0xb1, 0x44, 0x75,
      0x6d, 0x42, 0x89, 0xcc, 0x19, 0x14, 0x03, 0x4e,
      0x3c, 0x7b, 0x47, 0x6b, 0xa2, 0xff, 0xf0, 0x68,
      0xd4, 0xf2, 0x1b, 0xac, 0xbb, 0x37, 0xd6, 0x5e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ros_robot_controller_msgs__msg__RGBState__TYPE_NAME[] = "ros_robot_controller_msgs/msg/RGBState";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__msg__RGBState__FIELD_NAME__index[] = "index";
static char ros_robot_controller_msgs__msg__RGBState__FIELD_NAME__red[] = "red";
static char ros_robot_controller_msgs__msg__RGBState__FIELD_NAME__green[] = "green";
static char ros_robot_controller_msgs__msg__RGBState__FIELD_NAME__blue[] = "blue";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__msg__RGBState__FIELDS[] = {
  {
    {ros_robot_controller_msgs__msg__RGBState__FIELD_NAME__index, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__RGBState__FIELD_NAME__red, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__RGBState__FIELD_NAME__green, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__RGBState__FIELD_NAME__blue, 4, 4},
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
ros_robot_controller_msgs__msg__RGBState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__msg__RGBState__TYPE_NAME, 38, 38},
      {ros_robot_controller_msgs__msg__RGBState__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 index\n"
  "uint8 red\n"
  "uint8 green\n"
  "uint8 blue\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__RGBState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__msg__RGBState__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 46, 46},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__RGBState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__msg__RGBState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
