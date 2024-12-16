// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:msg/BusServoState.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/msg/detail/bus_servo_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__BusServoState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbd, 0x55, 0x56, 0xbe, 0xec, 0x88, 0x06, 0x95,
      0x5d, 0xd3, 0xfb, 0x44, 0x79, 0x68, 0xdf, 0x82,
      0x7d, 0x2f, 0x8a, 0x10, 0xbe, 0xf6, 0x4b, 0x01,
      0x59, 0x37, 0x1b, 0xb1, 0xad, 0xad, 0x62, 0x9f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ros_robot_controller_msgs__msg__BusServoState__TYPE_NAME[] = "ros_robot_controller_msgs/msg/BusServoState";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__present_id[] = "present_id";
static char ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__target_id[] = "target_id";
static char ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__position[] = "position";
static char ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__offset[] = "offset";
static char ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__voltage[] = "voltage";
static char ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__temperature[] = "temperature";
static char ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__position_limit[] = "position_limit";
static char ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__voltage_limit[] = "voltage_limit";
static char ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__max_temperature_limit[] = "max_temperature_limit";
static char ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__enable_torque[] = "enable_torque";
static char ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__save_offset[] = "save_offset";
static char ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__stop[] = "stop";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__msg__BusServoState__FIELDS[] = {
  {
    {ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__present_id, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__target_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__offset, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__voltage, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__temperature, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__position_limit, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__voltage_limit, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__max_temperature_limit, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__enable_torque, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__save_offset, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__BusServoState__FIELD_NAME__stop, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros_robot_controller_msgs__msg__BusServoState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__msg__BusServoState__TYPE_NAME, 43, 43},
      {ros_robot_controller_msgs__msg__BusServoState__FIELDS, 12, 12},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint16[] present_id\n"
  "uint16[] target_id\n"
  "uint16[] position\n"
  "int16[] offset\n"
  "uint16[] voltage\n"
  "uint16[] temperature\n"
  "uint16[] position_limit\n"
  "uint16[] voltage_limit\n"
  "uint16[] max_temperature_limit\n"
  "uint16[] enable_torque\n"
  "uint16[] save_offset\n"
  "uint16[] stop";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__BusServoState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__msg__BusServoState__TYPE_NAME, 43, 43},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 246, 246},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__BusServoState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__msg__BusServoState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
