// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:msg/ButtonState.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/msg/detail/button_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__ButtonState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xcf, 0x2c, 0x4e, 0x12, 0xe4, 0xf2, 0x5a, 0xcc,
      0x84, 0x9e, 0xd4, 0x84, 0xea, 0xcb, 0x5c, 0x20,
      0xbb, 0x06, 0xf3, 0xe2, 0x12, 0xd5, 0x07, 0x8c,
      0xbd, 0x2f, 0xdc, 0xeb, 0xd9, 0x2d, 0xf3, 0x45,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ros_robot_controller_msgs__msg__ButtonState__TYPE_NAME[] = "ros_robot_controller_msgs/msg/ButtonState";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__msg__ButtonState__FIELD_NAME__id[] = "id";
static char ros_robot_controller_msgs__msg__ButtonState__FIELD_NAME__state[] = "state";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__msg__ButtonState__FIELDS[] = {
  {
    {ros_robot_controller_msgs__msg__ButtonState__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__ButtonState__FIELD_NAME__state, 5, 5},
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
ros_robot_controller_msgs__msg__ButtonState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__msg__ButtonState__TYPE_NAME, 41, 41},
      {ros_robot_controller_msgs__msg__ButtonState__FIELDS, 2, 2},
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
  "uint8 state";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__ButtonState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__msg__ButtonState__TYPE_NAME, 41, 41},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 21, 21},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__ButtonState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__msg__ButtonState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
