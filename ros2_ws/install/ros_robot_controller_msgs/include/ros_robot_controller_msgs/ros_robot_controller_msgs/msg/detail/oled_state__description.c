// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:msg/OLEDState.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/msg/detail/oled_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__OLEDState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x59, 0x9b, 0x4e, 0xe7, 0x64, 0x5d, 0x45, 0x17,
      0x07, 0x58, 0x8e, 0x24, 0x43, 0x5c, 0xd8, 0x24,
      0xa8, 0x38, 0x35, 0xee, 0xf4, 0x05, 0x26, 0x46,
      0x72, 0x98, 0x41, 0x2f, 0x78, 0x62, 0x40, 0x2d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ros_robot_controller_msgs__msg__OLEDState__TYPE_NAME[] = "ros_robot_controller_msgs/msg/OLEDState";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__msg__OLEDState__FIELD_NAME__index[] = "index";
static char ros_robot_controller_msgs__msg__OLEDState__FIELD_NAME__text[] = "text";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__msg__OLEDState__FIELDS[] = {
  {
    {ros_robot_controller_msgs__msg__OLEDState__FIELD_NAME__index, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__OLEDState__FIELD_NAME__text, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros_robot_controller_msgs__msg__OLEDState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__msg__OLEDState__TYPE_NAME, 39, 39},
      {ros_robot_controller_msgs__msg__OLEDState__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 index\n"
  "string text";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__OLEDState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__msg__OLEDState__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 24, 24},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__OLEDState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__msg__OLEDState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
