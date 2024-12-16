// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:msg/GetBusServoCmd.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/msg/detail/get_bus_servo_cmd__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__GetBusServoCmd__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x23, 0xbe, 0xbb, 0xe6, 0x86, 0xec, 0x0e, 0xad,
      0x41, 0x87, 0xb9, 0x1c, 0x94, 0x66, 0x46, 0x7f,
      0x74, 0x44, 0x60, 0x31, 0x0d, 0xaa, 0x1d, 0x72,
      0xc8, 0x1d, 0x60, 0x59, 0xb9, 0x0f, 0xdf, 0x78,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ros_robot_controller_msgs__msg__GetBusServoCmd__TYPE_NAME[] = "ros_robot_controller_msgs/msg/GetBusServoCmd";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__id[] = "id";
static char ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_id[] = "get_id";
static char ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_position[] = "get_position";
static char ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_offset[] = "get_offset";
static char ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_voltage[] = "get_voltage";
static char ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_temperature[] = "get_temperature";
static char ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_position_limit[] = "get_position_limit";
static char ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_voltage_limit[] = "get_voltage_limit";
static char ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_max_temperature_limit[] = "get_max_temperature_limit";
static char ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_torque_state[] = "get_torque_state";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__msg__GetBusServoCmd__FIELDS[] = {
  {
    {ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_id, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_position, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_offset, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_voltage, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_temperature, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_position_limit, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_voltage_limit, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_max_temperature_limit, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__GetBusServoCmd__FIELD_NAME__get_torque_state, 16, 16},
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
ros_robot_controller_msgs__msg__GetBusServoCmd__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__msg__GetBusServoCmd__TYPE_NAME, 44, 44},
      {ros_robot_controller_msgs__msg__GetBusServoCmd__FIELDS, 10, 10},
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
  "uint8 get_id\n"
  "uint8 get_position\n"
  "uint8 get_offset\n"
  "uint8 get_voltage\n"
  "uint8 get_temperature\n"
  "uint8 get_position_limit\n"
  "uint8 get_voltage_limit\n"
  "uint8 get_max_temperature_limit\n"
  "uint8 get_torque_state";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__GetBusServoCmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__msg__GetBusServoCmd__TYPE_NAME, 44, 44},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 202, 202},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__GetBusServoCmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__msg__GetBusServoCmd__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
