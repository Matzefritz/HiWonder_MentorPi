// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:msg/PWMServoState.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/msg/detail/pwm_servo_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__PWMServoState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd2, 0xda, 0x03, 0x83, 0x9b, 0x88, 0x3c, 0xda,
      0x85, 0x0a, 0x30, 0x65, 0x76, 0xf5, 0xe2, 0xb4,
      0xcf, 0x47, 0x07, 0xcf, 0x37, 0x18, 0x43, 0x33,
      0x10, 0x5e, 0xb0, 0x49, 0xa5, 0x83, 0x99, 0x6d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ros_robot_controller_msgs__msg__PWMServoState__TYPE_NAME[] = "ros_robot_controller_msgs/msg/PWMServoState";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__msg__PWMServoState__FIELD_NAME__id[] = "id";
static char ros_robot_controller_msgs__msg__PWMServoState__FIELD_NAME__position[] = "position";
static char ros_robot_controller_msgs__msg__PWMServoState__FIELD_NAME__offset[] = "offset";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__msg__PWMServoState__FIELDS[] = {
  {
    {ros_robot_controller_msgs__msg__PWMServoState__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__PWMServoState__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__PWMServoState__FIELD_NAME__offset, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros_robot_controller_msgs__msg__PWMServoState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__msg__PWMServoState__TYPE_NAME, 43, 43},
      {ros_robot_controller_msgs__msg__PWMServoState__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint16[] id\n"
  "uint16[] position\n"
  "int16[] offset";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__PWMServoState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__msg__PWMServoState__TYPE_NAME, 43, 43},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 45, 45},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__PWMServoState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__msg__PWMServoState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
