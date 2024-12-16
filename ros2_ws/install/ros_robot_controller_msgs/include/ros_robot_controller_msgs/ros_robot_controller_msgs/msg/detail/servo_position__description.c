// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:msg/ServoPosition.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/msg/detail/servo_position__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__ServoPosition__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd4, 0xf6, 0x93, 0xb5, 0x2a, 0x38, 0x7f, 0x96,
      0x1e, 0x98, 0x90, 0x60, 0x2f, 0x63, 0x3b, 0xa0,
      0x29, 0x5f, 0x53, 0x53, 0xd1, 0x65, 0x82, 0xf4,
      0xec, 0xb1, 0x09, 0x78, 0x5b, 0x03, 0xf2, 0xb4,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ros_robot_controller_msgs__msg__ServoPosition__TYPE_NAME[] = "ros_robot_controller_msgs/msg/ServoPosition";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__msg__ServoPosition__FIELD_NAME__id[] = "id";
static char ros_robot_controller_msgs__msg__ServoPosition__FIELD_NAME__position[] = "position";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__msg__ServoPosition__FIELDS[] = {
  {
    {ros_robot_controller_msgs__msg__ServoPosition__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__ServoPosition__FIELD_NAME__position, 8, 8},
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
ros_robot_controller_msgs__msg__ServoPosition__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__msg__ServoPosition__TYPE_NAME, 43, 43},
      {ros_robot_controller_msgs__msg__ServoPosition__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint16 id\n"
  "uint16 position";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__ServoPosition__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__msg__ServoPosition__TYPE_NAME, 43, 43},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 26, 26},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__ServoPosition__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__msg__ServoPosition__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
