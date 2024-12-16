// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:msg/RGBStates.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/msg/detail/rgb_states__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__RGBStates__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4b, 0xae, 0x93, 0xc1, 0x14, 0x39, 0x9d, 0x81,
      0x44, 0x17, 0xb1, 0xce, 0xdb, 0xce, 0xa3, 0x24,
      0x5e, 0xcb, 0xe6, 0xc5, 0x28, 0x9d, 0x1a, 0x9e,
      0x73, 0x3e, 0x2a, 0x30, 0x63, 0x99, 0x27, 0x2e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "ros_robot_controller_msgs/msg/detail/rgb_state__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t ros_robot_controller_msgs__msg__RGBState__EXPECTED_HASH = {1, {
    0xde, 0x85, 0xae, 0x06, 0x06, 0xb1, 0x44, 0x75,
    0x6d, 0x42, 0x89, 0xcc, 0x19, 0x14, 0x03, 0x4e,
    0x3c, 0x7b, 0x47, 0x6b, 0xa2, 0xff, 0xf0, 0x68,
    0xd4, 0xf2, 0x1b, 0xac, 0xbb, 0x37, 0xd6, 0x5e,
  }};
#endif

static char ros_robot_controller_msgs__msg__RGBStates__TYPE_NAME[] = "ros_robot_controller_msgs/msg/RGBStates";
static char ros_robot_controller_msgs__msg__RGBState__TYPE_NAME[] = "ros_robot_controller_msgs/msg/RGBState";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__msg__RGBStates__FIELD_NAME__states[] = "states";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__msg__RGBStates__FIELDS[] = {
  {
    {ros_robot_controller_msgs__msg__RGBStates__FIELD_NAME__states, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {ros_robot_controller_msgs__msg__RGBState__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ros_robot_controller_msgs__msg__RGBStates__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {ros_robot_controller_msgs__msg__RGBState__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros_robot_controller_msgs__msg__RGBStates__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__msg__RGBStates__TYPE_NAME, 39, 39},
      {ros_robot_controller_msgs__msg__RGBStates__FIELDS, 1, 1},
    },
    {ros_robot_controller_msgs__msg__RGBStates__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&ros_robot_controller_msgs__msg__RGBState__EXPECTED_HASH, ros_robot_controller_msgs__msg__RGBState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = ros_robot_controller_msgs__msg__RGBState__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "RGBState[] states";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__RGBStates__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__msg__RGBStates__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 18, 18},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__RGBStates__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__msg__RGBStates__get_individual_type_description_source(NULL),
    sources[1] = *ros_robot_controller_msgs__msg__RGBState__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
