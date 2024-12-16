// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:msg/ServosPosition.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/msg/detail/servos_position__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__ServosPosition__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x58, 0xef, 0xf5, 0x3e, 0x9b, 0xf7, 0xbb, 0xb9,
      0xc2, 0xe1, 0xe3, 0xd3, 0x6c, 0x9f, 0xc1, 0x35,
      0xb0, 0xa4, 0x82, 0xa9, 0xce, 0x96, 0x57, 0x7b,
      0x85, 0xa1, 0x3d, 0x78, 0x06, 0xae, 0x61, 0x3d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "ros_robot_controller_msgs/msg/detail/servo_position__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t ros_robot_controller_msgs__msg__ServoPosition__EXPECTED_HASH = {1, {
    0xd4, 0xf6, 0x93, 0xb5, 0x2a, 0x38, 0x7f, 0x96,
    0x1e, 0x98, 0x90, 0x60, 0x2f, 0x63, 0x3b, 0xa0,
    0x29, 0x5f, 0x53, 0x53, 0xd1, 0x65, 0x82, 0xf4,
    0xec, 0xb1, 0x09, 0x78, 0x5b, 0x03, 0xf2, 0xb4,
  }};
#endif

static char ros_robot_controller_msgs__msg__ServosPosition__TYPE_NAME[] = "ros_robot_controller_msgs/msg/ServosPosition";
static char ros_robot_controller_msgs__msg__ServoPosition__TYPE_NAME[] = "ros_robot_controller_msgs/msg/ServoPosition";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__msg__ServosPosition__FIELD_NAME__duration[] = "duration";
static char ros_robot_controller_msgs__msg__ServosPosition__FIELD_NAME__position[] = "position";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__msg__ServosPosition__FIELDS[] = {
  {
    {ros_robot_controller_msgs__msg__ServosPosition__FIELD_NAME__duration, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__ServosPosition__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {ros_robot_controller_msgs__msg__ServoPosition__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ros_robot_controller_msgs__msg__ServosPosition__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {ros_robot_controller_msgs__msg__ServoPosition__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros_robot_controller_msgs__msg__ServosPosition__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__msg__ServosPosition__TYPE_NAME, 44, 44},
      {ros_robot_controller_msgs__msg__ServosPosition__FIELDS, 2, 2},
    },
    {ros_robot_controller_msgs__msg__ServosPosition__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&ros_robot_controller_msgs__msg__ServoPosition__EXPECTED_HASH, ros_robot_controller_msgs__msg__ServoPosition__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = ros_robot_controller_msgs__msg__ServoPosition__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 duration\n"
  "ros_robot_controller_msgs/ServoPosition[] position";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__ServosPosition__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__msg__ServosPosition__TYPE_NAME, 44, 44},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 68, 68},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__ServosPosition__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__msg__ServosPosition__get_individual_type_description_source(NULL),
    sources[1] = *ros_robot_controller_msgs__msg__ServoPosition__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
