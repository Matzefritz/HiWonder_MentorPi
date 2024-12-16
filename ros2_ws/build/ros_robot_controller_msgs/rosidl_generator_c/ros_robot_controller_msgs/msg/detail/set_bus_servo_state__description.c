// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:msg/SetBusServoState.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/msg/detail/set_bus_servo_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__SetBusServoState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x71, 0xc0, 0x9c, 0x08, 0xf3, 0xff, 0xc7, 0x15,
      0x11, 0x34, 0xba, 0x38, 0xc5, 0x98, 0xb3, 0x23,
      0xb0, 0xb1, 0x98, 0xfa, 0x80, 0x44, 0xec, 0x64,
      0xab, 0x0a, 0x55, 0x1c, 0xdb, 0x7e, 0x63, 0x25,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "ros_robot_controller_msgs/msg/detail/bus_servo_state__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t ros_robot_controller_msgs__msg__BusServoState__EXPECTED_HASH = {1, {
    0xbd, 0x55, 0x56, 0xbe, 0xec, 0x88, 0x06, 0x95,
    0x5d, 0xd3, 0xfb, 0x44, 0x79, 0x68, 0xdf, 0x82,
    0x7d, 0x2f, 0x8a, 0x10, 0xbe, 0xf6, 0x4b, 0x01,
    0x59, 0x37, 0x1b, 0xb1, 0xad, 0xad, 0x62, 0x9f,
  }};
#endif

static char ros_robot_controller_msgs__msg__SetBusServoState__TYPE_NAME[] = "ros_robot_controller_msgs/msg/SetBusServoState";
static char ros_robot_controller_msgs__msg__BusServoState__TYPE_NAME[] = "ros_robot_controller_msgs/msg/BusServoState";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__msg__SetBusServoState__FIELD_NAME__state[] = "state";
static char ros_robot_controller_msgs__msg__SetBusServoState__FIELD_NAME__duration[] = "duration";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__msg__SetBusServoState__FIELDS[] = {
  {
    {ros_robot_controller_msgs__msg__SetBusServoState__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {ros_robot_controller_msgs__msg__BusServoState__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__SetBusServoState__FIELD_NAME__duration, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ros_robot_controller_msgs__msg__SetBusServoState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {ros_robot_controller_msgs__msg__BusServoState__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros_robot_controller_msgs__msg__SetBusServoState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__msg__SetBusServoState__TYPE_NAME, 46, 46},
      {ros_robot_controller_msgs__msg__SetBusServoState__FIELDS, 2, 2},
    },
    {ros_robot_controller_msgs__msg__SetBusServoState__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&ros_robot_controller_msgs__msg__BusServoState__EXPECTED_HASH, ros_robot_controller_msgs__msg__BusServoState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = ros_robot_controller_msgs__msg__BusServoState__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "ros_robot_controller_msgs/BusServoState[] state\n"
  "\n"
  "float64 duration";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__SetBusServoState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__msg__SetBusServoState__TYPE_NAME, 46, 46},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 66, 66},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__SetBusServoState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__msg__SetBusServoState__get_individual_type_description_source(NULL),
    sources[1] = *ros_robot_controller_msgs__msg__BusServoState__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
