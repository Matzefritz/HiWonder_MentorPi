// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ros_robot_controller_msgs:msg/LedState.idl
// generated code does not contain a copyright notice
#include "ros_robot_controller_msgs/msg/detail/led_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ros_robot_controller_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros_robot_controller_msgs/msg/detail/led_state__struct.h"
#include "ros_robot_controller_msgs/msg/detail/led_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _LedState__ros_msg_type = ros_robot_controller_msgs__msg__LedState;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_robot_controller_msgs
bool cdr_serialize_ros_robot_controller_msgs__msg__LedState(
  const ros_robot_controller_msgs__msg__LedState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: on_time
  {
    cdr << ros_message->on_time;
  }

  // Field name: off_time
  {
    cdr << ros_message->off_time;
  }

  // Field name: repeat
  {
    cdr << ros_message->repeat;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_robot_controller_msgs
bool cdr_deserialize_ros_robot_controller_msgs__msg__LedState(
  eprosima::fastcdr::Cdr & cdr,
  ros_robot_controller_msgs__msg__LedState * ros_message)
{
  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: on_time
  {
    cdr >> ros_message->on_time;
  }

  // Field name: off_time
  {
    cdr >> ros_message->off_time;
  }

  // Field name: repeat
  {
    cdr >> ros_message->repeat;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_robot_controller_msgs
size_t get_serialized_size_ros_robot_controller_msgs__msg__LedState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LedState__ros_msg_type * ros_message = static_cast<const _LedState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: on_time
  {
    size_t item_size = sizeof(ros_message->on_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: off_time
  {
    size_t item_size = sizeof(ros_message->off_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: repeat
  {
    size_t item_size = sizeof(ros_message->repeat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_robot_controller_msgs
size_t max_serialized_size_ros_robot_controller_msgs__msg__LedState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: id
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: on_time
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: off_time
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: repeat
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros_robot_controller_msgs__msg__LedState;
    is_plain =
      (
      offsetof(DataType, repeat) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_robot_controller_msgs
bool cdr_serialize_key_ros_robot_controller_msgs__msg__LedState(
  const ros_robot_controller_msgs__msg__LedState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: on_time
  {
    cdr << ros_message->on_time;
  }

  // Field name: off_time
  {
    cdr << ros_message->off_time;
  }

  // Field name: repeat
  {
    cdr << ros_message->repeat;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_robot_controller_msgs
size_t get_serialized_size_key_ros_robot_controller_msgs__msg__LedState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LedState__ros_msg_type * ros_message = static_cast<const _LedState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: on_time
  {
    size_t item_size = sizeof(ros_message->on_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: off_time
  {
    size_t item_size = sizeof(ros_message->off_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: repeat
  {
    size_t item_size = sizeof(ros_message->repeat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_robot_controller_msgs
size_t max_serialized_size_key_ros_robot_controller_msgs__msg__LedState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: id
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: on_time
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: off_time
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: repeat
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros_robot_controller_msgs__msg__LedState;
    is_plain =
      (
      offsetof(DataType, repeat) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _LedState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const ros_robot_controller_msgs__msg__LedState * ros_message = static_cast<const ros_robot_controller_msgs__msg__LedState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_ros_robot_controller_msgs__msg__LedState(ros_message, cdr);
}

static bool _LedState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  ros_robot_controller_msgs__msg__LedState * ros_message = static_cast<ros_robot_controller_msgs__msg__LedState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_ros_robot_controller_msgs__msg__LedState(cdr, ros_message);
}

static uint32_t _LedState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros_robot_controller_msgs__msg__LedState(
      untyped_ros_message, 0));
}

static size_t _LedState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros_robot_controller_msgs__msg__LedState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_LedState = {
  "ros_robot_controller_msgs::msg",
  "LedState",
  _LedState__cdr_serialize,
  _LedState__cdr_deserialize,
  _LedState__get_serialized_size,
  _LedState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _LedState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LedState,
  get_message_typesupport_handle_function,
  &ros_robot_controller_msgs__msg__LedState__get_type_hash,
  &ros_robot_controller_msgs__msg__LedState__get_type_description,
  &ros_robot_controller_msgs__msg__LedState__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros_robot_controller_msgs, msg, LedState)() {
  return &_LedState__type_support;
}

#if defined(__cplusplus)
}
#endif