// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:msg/MotorsState.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/msg/detail/motors_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__MotorsState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5c, 0x66, 0x08, 0xfd, 0x2c, 0x4a, 0x57, 0xd9,
      0x0e, 0x50, 0xe2, 0xec, 0xc7, 0x81, 0x0e, 0x1a,
      0x97, 0xb1, 0x55, 0x90, 0x56, 0x95, 0x1b, 0x3f,
      0x21, 0xac, 0x96, 0xc0, 0x87, 0x44, 0xa1, 0xef,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "ros_robot_controller_msgs/msg/detail/motor_state__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t ros_robot_controller_msgs__msg__MotorState__EXPECTED_HASH = {1, {
    0xb2, 0x24, 0x89, 0x7b, 0x01, 0x8b, 0x13, 0xe7,
    0xde, 0xc8, 0xf8, 0x03, 0x45, 0x79, 0x96, 0x43,
    0x29, 0x53, 0xbc, 0x42, 0x92, 0x2e, 0xc9, 0xa3,
    0x05, 0x01, 0xf4, 0x5e, 0x4f, 0xcc, 0x61, 0x4e,
  }};
#endif

static char ros_robot_controller_msgs__msg__MotorsState__TYPE_NAME[] = "ros_robot_controller_msgs/msg/MotorsState";
static char ros_robot_controller_msgs__msg__MotorState__TYPE_NAME[] = "ros_robot_controller_msgs/msg/MotorState";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__msg__MotorsState__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__msg__MotorsState__FIELDS[] = {
  {
    {ros_robot_controller_msgs__msg__MotorsState__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {ros_robot_controller_msgs__msg__MotorState__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ros_robot_controller_msgs__msg__MotorsState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {ros_robot_controller_msgs__msg__MotorState__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros_robot_controller_msgs__msg__MotorsState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__msg__MotorsState__TYPE_NAME, 41, 41},
      {ros_robot_controller_msgs__msg__MotorsState__FIELDS, 1, 1},
    },
    {ros_robot_controller_msgs__msg__MotorsState__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&ros_robot_controller_msgs__msg__MotorState__EXPECTED_HASH, ros_robot_controller_msgs__msg__MotorState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = ros_robot_controller_msgs__msg__MotorState__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "ros_robot_controller_msgs/MotorState[] data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__MotorsState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__msg__MotorsState__TYPE_NAME, 41, 41},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 44, 44},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__MotorsState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__msg__MotorsState__get_individual_type_description_source(NULL),
    sources[1] = *ros_robot_controller_msgs__msg__MotorState__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
