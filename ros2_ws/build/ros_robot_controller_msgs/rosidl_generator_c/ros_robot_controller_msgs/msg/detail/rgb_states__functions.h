// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ros_robot_controller_msgs:msg/RGBStates.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/rgb_states.h"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__RGB_STATES__FUNCTIONS_H_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__RGB_STATES__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "ros_robot_controller_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ros_robot_controller_msgs/msg/detail/rgb_states__struct.h"

/// Initialize msg/RGBStates message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ros_robot_controller_msgs__msg__RGBStates
 * )) before or use
 * ros_robot_controller_msgs__msg__RGBStates__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
bool
ros_robot_controller_msgs__msg__RGBStates__init(ros_robot_controller_msgs__msg__RGBStates * msg);

/// Finalize msg/RGBStates message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
void
ros_robot_controller_msgs__msg__RGBStates__fini(ros_robot_controller_msgs__msg__RGBStates * msg);

/// Create msg/RGBStates message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ros_robot_controller_msgs__msg__RGBStates__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
ros_robot_controller_msgs__msg__RGBStates *
ros_robot_controller_msgs__msg__RGBStates__create(void);

/// Destroy msg/RGBStates message.
/**
 * It calls
 * ros_robot_controller_msgs__msg__RGBStates__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
void
ros_robot_controller_msgs__msg__RGBStates__destroy(ros_robot_controller_msgs__msg__RGBStates * msg);

/// Check for msg/RGBStates message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
bool
ros_robot_controller_msgs__msg__RGBStates__are_equal(const ros_robot_controller_msgs__msg__RGBStates * lhs, const ros_robot_controller_msgs__msg__RGBStates * rhs);

/// Copy a msg/RGBStates message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
bool
ros_robot_controller_msgs__msg__RGBStates__copy(
  const ros_robot_controller_msgs__msg__RGBStates * input,
  ros_robot_controller_msgs__msg__RGBStates * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_type_hash_t *
ros_robot_controller_msgs__msg__RGBStates__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_runtime_c__type_description__TypeDescription *
ros_robot_controller_msgs__msg__RGBStates__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_runtime_c__type_description__TypeSource *
ros_robot_controller_msgs__msg__RGBStates__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros_robot_controller_msgs__msg__RGBStates__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/RGBStates messages.
/**
 * It allocates the memory for the number of elements and calls
 * ros_robot_controller_msgs__msg__RGBStates__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
bool
ros_robot_controller_msgs__msg__RGBStates__Sequence__init(ros_robot_controller_msgs__msg__RGBStates__Sequence * array, size_t size);

/// Finalize array of msg/RGBStates messages.
/**
 * It calls
 * ros_robot_controller_msgs__msg__RGBStates__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
void
ros_robot_controller_msgs__msg__RGBStates__Sequence__fini(ros_robot_controller_msgs__msg__RGBStates__Sequence * array);

/// Create array of msg/RGBStates messages.
/**
 * It allocates the memory for the array and calls
 * ros_robot_controller_msgs__msg__RGBStates__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
ros_robot_controller_msgs__msg__RGBStates__Sequence *
ros_robot_controller_msgs__msg__RGBStates__Sequence__create(size_t size);

/// Destroy array of msg/RGBStates messages.
/**
 * It calls
 * ros_robot_controller_msgs__msg__RGBStates__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
void
ros_robot_controller_msgs__msg__RGBStates__Sequence__destroy(ros_robot_controller_msgs__msg__RGBStates__Sequence * array);

/// Check for msg/RGBStates message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
bool
ros_robot_controller_msgs__msg__RGBStates__Sequence__are_equal(const ros_robot_controller_msgs__msg__RGBStates__Sequence * lhs, const ros_robot_controller_msgs__msg__RGBStates__Sequence * rhs);

/// Copy an array of msg/RGBStates messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_robot_controller_msgs
bool
ros_robot_controller_msgs__msg__RGBStates__Sequence__copy(
  const ros_robot_controller_msgs__msg__RGBStates__Sequence * input,
  ros_robot_controller_msgs__msg__RGBStates__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__RGB_STATES__FUNCTIONS_H_
