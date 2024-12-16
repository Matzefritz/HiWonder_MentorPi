// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros_robot_controller_msgs:srv/GetPWMServoState.idl
// generated code does not contain a copyright notice

#include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__srv__GetPWMServoState__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xda, 0x15, 0x16, 0x52, 0x95, 0x71, 0xcb, 0x4f,
      0x40, 0x90, 0x8b, 0x7a, 0x6c, 0x18, 0x7c, 0x74,
      0x00, 0xd6, 0xa0, 0x52, 0xa0, 0x6b, 0x71, 0x86,
      0xab, 0x1c, 0xae, 0x4d, 0x4d, 0x19, 0x4a, 0x07,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x57, 0xa1, 0x78, 0x18, 0xb3, 0xad, 0x07, 0xb6,
      0xbd, 0xd5, 0x0d, 0x07, 0x3c, 0xf0, 0x9a, 0x43,
      0xb3, 0x61, 0x9b, 0xb5, 0x23, 0x12, 0x59, 0x04,
      0x5e, 0x9c, 0xb0, 0x0a, 0x2a, 0x93, 0xa4, 0xff,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3e, 0xa9, 0x51, 0x37, 0xe8, 0xc9, 0x57, 0xf4,
      0x5d, 0x43, 0x62, 0x00, 0x67, 0xa5, 0x63, 0xb3,
      0x3a, 0xbe, 0x95, 0x2c, 0x54, 0x85, 0x0e, 0x5e,
      0x19, 0x8a, 0x8a, 0x1e, 0x0d, 0x5f, 0x72, 0x74,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x41, 0x46, 0x0d, 0xf5, 0x9f, 0x41, 0x84, 0x07,
      0x35, 0xff, 0xa0, 0xf9, 0x40, 0x9c, 0xae, 0x2e,
      0x69, 0x51, 0xce, 0x98, 0xff, 0x71, 0xa7, 0xaf,
      0xa0, 0xcc, 0x26, 0x34, 0x40, 0x42, 0x25, 0x37,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "ros_robot_controller_msgs/msg/detail/get_pwm_servo_cmd__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "ros_robot_controller_msgs/msg/detail/pwm_servo_state__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t ros_robot_controller_msgs__msg__GetPWMServoCmd__EXPECTED_HASH = {1, {
    0xe3, 0xe0, 0xa0, 0x6e, 0xd3, 0xf4, 0x9c, 0x40,
    0x0d, 0x6e, 0x24, 0xfa, 0x5a, 0x9e, 0xc3, 0x8a,
    0xe8, 0x0d, 0xe3, 0x64, 0x27, 0x87, 0xcb, 0x3c,
    0xaa, 0x06, 0x10, 0x8c, 0xbc, 0x93, 0xf4, 0xc4,
  }};
static const rosidl_type_hash_t ros_robot_controller_msgs__msg__PWMServoState__EXPECTED_HASH = {1, {
    0xd2, 0xda, 0x03, 0x83, 0x9b, 0x88, 0x3c, 0xda,
    0x85, 0x0a, 0x30, 0x65, 0x76, 0xf5, 0xe2, 0xb4,
    0xcf, 0x47, 0x07, 0xcf, 0x37, 0x18, 0x43, 0x33,
    0x10, 0x5e, 0xb0, 0x49, 0xa5, 0x83, 0x99, 0x6d,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char ros_robot_controller_msgs__srv__GetPWMServoState__TYPE_NAME[] = "ros_robot_controller_msgs/srv/GetPWMServoState";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char ros_robot_controller_msgs__msg__GetPWMServoCmd__TYPE_NAME[] = "ros_robot_controller_msgs/msg/GetPWMServoCmd";
static char ros_robot_controller_msgs__msg__PWMServoState__TYPE_NAME[] = "ros_robot_controller_msgs/msg/PWMServoState";
static char ros_robot_controller_msgs__srv__GetPWMServoState_Event__TYPE_NAME[] = "ros_robot_controller_msgs/srv/GetPWMServoState_Event";
static char ros_robot_controller_msgs__srv__GetPWMServoState_Request__TYPE_NAME[] = "ros_robot_controller_msgs/srv/GetPWMServoState_Request";
static char ros_robot_controller_msgs__srv__GetPWMServoState_Response__TYPE_NAME[] = "ros_robot_controller_msgs/srv/GetPWMServoState_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char ros_robot_controller_msgs__srv__GetPWMServoState__FIELD_NAME__request_message[] = "request_message";
static char ros_robot_controller_msgs__srv__GetPWMServoState__FIELD_NAME__response_message[] = "response_message";
static char ros_robot_controller_msgs__srv__GetPWMServoState__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__srv__GetPWMServoState__FIELDS[] = {
  {
    {ros_robot_controller_msgs__srv__GetPWMServoState__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {ros_robot_controller_msgs__srv__GetPWMServoState_Request__TYPE_NAME, 54, 54},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__srv__GetPWMServoState__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {ros_robot_controller_msgs__srv__GetPWMServoState_Response__TYPE_NAME, 55, 55},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__srv__GetPWMServoState__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {ros_robot_controller_msgs__srv__GetPWMServoState_Event__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ros_robot_controller_msgs__srv__GetPWMServoState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__GetPWMServoCmd__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__PWMServoState__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__srv__GetPWMServoState_Event__TYPE_NAME, 52, 52},
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__srv__GetPWMServoState_Request__TYPE_NAME, 54, 54},
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__srv__GetPWMServoState_Response__TYPE_NAME, 55, 55},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros_robot_controller_msgs__srv__GetPWMServoState__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__srv__GetPWMServoState__TYPE_NAME, 46, 46},
      {ros_robot_controller_msgs__srv__GetPWMServoState__FIELDS, 3, 3},
    },
    {ros_robot_controller_msgs__srv__GetPWMServoState__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&ros_robot_controller_msgs__msg__GetPWMServoCmd__EXPECTED_HASH, ros_robot_controller_msgs__msg__GetPWMServoCmd__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = ros_robot_controller_msgs__msg__GetPWMServoCmd__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&ros_robot_controller_msgs__msg__PWMServoState__EXPECTED_HASH, ros_robot_controller_msgs__msg__PWMServoState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = ros_robot_controller_msgs__msg__PWMServoState__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char ros_robot_controller_msgs__srv__GetPWMServoState_Request__FIELD_NAME__cmd[] = "cmd";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__srv__GetPWMServoState_Request__FIELDS[] = {
  {
    {ros_robot_controller_msgs__srv__GetPWMServoState_Request__FIELD_NAME__cmd, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {ros_robot_controller_msgs__msg__GetPWMServoCmd__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ros_robot_controller_msgs__srv__GetPWMServoState_Request__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {ros_robot_controller_msgs__msg__GetPWMServoCmd__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__srv__GetPWMServoState_Request__TYPE_NAME, 54, 54},
      {ros_robot_controller_msgs__srv__GetPWMServoState_Request__FIELDS, 1, 1},
    },
    {ros_robot_controller_msgs__srv__GetPWMServoState_Request__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&ros_robot_controller_msgs__msg__GetPWMServoCmd__EXPECTED_HASH, ros_robot_controller_msgs__msg__GetPWMServoCmd__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = ros_robot_controller_msgs__msg__GetPWMServoCmd__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char ros_robot_controller_msgs__srv__GetPWMServoState_Response__FIELD_NAME__success[] = "success";
static char ros_robot_controller_msgs__srv__GetPWMServoState_Response__FIELD_NAME__state[] = "state";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__srv__GetPWMServoState_Response__FIELDS[] = {
  {
    {ros_robot_controller_msgs__srv__GetPWMServoState_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__srv__GetPWMServoState_Response__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {ros_robot_controller_msgs__msg__PWMServoState__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ros_robot_controller_msgs__srv__GetPWMServoState_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {ros_robot_controller_msgs__msg__PWMServoState__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__srv__GetPWMServoState_Response__TYPE_NAME, 55, 55},
      {ros_robot_controller_msgs__srv__GetPWMServoState_Response__FIELDS, 2, 2},
    },
    {ros_robot_controller_msgs__srv__GetPWMServoState_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&ros_robot_controller_msgs__msg__PWMServoState__EXPECTED_HASH, ros_robot_controller_msgs__msg__PWMServoState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = ros_robot_controller_msgs__msg__PWMServoState__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char ros_robot_controller_msgs__srv__GetPWMServoState_Event__FIELD_NAME__info[] = "info";
static char ros_robot_controller_msgs__srv__GetPWMServoState_Event__FIELD_NAME__request[] = "request";
static char ros_robot_controller_msgs__srv__GetPWMServoState_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field ros_robot_controller_msgs__srv__GetPWMServoState_Event__FIELDS[] = {
  {
    {ros_robot_controller_msgs__srv__GetPWMServoState_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__srv__GetPWMServoState_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {ros_robot_controller_msgs__srv__GetPWMServoState_Request__TYPE_NAME, 54, 54},
    },
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__srv__GetPWMServoState_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {ros_robot_controller_msgs__srv__GetPWMServoState_Response__TYPE_NAME, 55, 55},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ros_robot_controller_msgs__srv__GetPWMServoState_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__GetPWMServoCmd__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__msg__PWMServoState__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__srv__GetPWMServoState_Request__TYPE_NAME, 54, 54},
    {NULL, 0, 0},
  },
  {
    {ros_robot_controller_msgs__srv__GetPWMServoState_Response__TYPE_NAME, 55, 55},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros_robot_controller_msgs__srv__GetPWMServoState_Event__TYPE_NAME, 52, 52},
      {ros_robot_controller_msgs__srv__GetPWMServoState_Event__FIELDS, 3, 3},
    },
    {ros_robot_controller_msgs__srv__GetPWMServoState_Event__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&ros_robot_controller_msgs__msg__GetPWMServoCmd__EXPECTED_HASH, ros_robot_controller_msgs__msg__GetPWMServoCmd__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = ros_robot_controller_msgs__msg__GetPWMServoCmd__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&ros_robot_controller_msgs__msg__PWMServoState__EXPECTED_HASH, ros_robot_controller_msgs__msg__PWMServoState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = ros_robot_controller_msgs__msg__PWMServoState__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "ros_robot_controller_msgs/GetPWMServoCmd[] cmd\n"
  "---\n"
  "bool success\n"
  "ros_robot_controller_msgs/PWMServoState[] state";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__srv__GetPWMServoState__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__srv__GetPWMServoState__TYPE_NAME, 46, 46},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 112, 112},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__srv__GetPWMServoState_Request__TYPE_NAME, 54, 54},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__srv__GetPWMServoState_Response__TYPE_NAME, 55, 55},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros_robot_controller_msgs__srv__GetPWMServoState_Event__TYPE_NAME, 52, 52},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__srv__GetPWMServoState__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__srv__GetPWMServoState__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *ros_robot_controller_msgs__msg__GetPWMServoCmd__get_individual_type_description_source(NULL);
    sources[3] = *ros_robot_controller_msgs__msg__PWMServoState__get_individual_type_description_source(NULL);
    sources[4] = *ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_individual_type_description_source(NULL);
    sources[5] = *ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_individual_type_description_source(NULL);
    sources[6] = *ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_individual_type_description_source(NULL);
    sources[7] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_individual_type_description_source(NULL),
    sources[1] = *ros_robot_controller_msgs__msg__GetPWMServoCmd__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_individual_type_description_source(NULL),
    sources[1] = *ros_robot_controller_msgs__msg__PWMServoState__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros_robot_controller_msgs__srv__GetPWMServoState_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *ros_robot_controller_msgs__msg__GetPWMServoCmd__get_individual_type_description_source(NULL);
    sources[3] = *ros_robot_controller_msgs__msg__PWMServoState__get_individual_type_description_source(NULL);
    sources[4] = *ros_robot_controller_msgs__srv__GetPWMServoState_Request__get_individual_type_description_source(NULL);
    sources[5] = *ros_robot_controller_msgs__srv__GetPWMServoState_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
