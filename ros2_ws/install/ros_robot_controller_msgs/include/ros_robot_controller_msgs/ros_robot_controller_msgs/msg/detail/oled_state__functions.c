// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_robot_controller_msgs:msg/OLEDState.idl
// generated code does not contain a copyright notice
#include "ros_robot_controller_msgs/msg/detail/oled_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `text`
#include "rosidl_runtime_c/string_functions.h"

bool
ros_robot_controller_msgs__msg__OLEDState__init(ros_robot_controller_msgs__msg__OLEDState * msg)
{
  if (!msg) {
    return false;
  }
  // index
  // text
  if (!rosidl_runtime_c__String__init(&msg->text)) {
    ros_robot_controller_msgs__msg__OLEDState__fini(msg);
    return false;
  }
  return true;
}

void
ros_robot_controller_msgs__msg__OLEDState__fini(ros_robot_controller_msgs__msg__OLEDState * msg)
{
  if (!msg) {
    return;
  }
  // index
  // text
  rosidl_runtime_c__String__fini(&msg->text);
}

bool
ros_robot_controller_msgs__msg__OLEDState__are_equal(const ros_robot_controller_msgs__msg__OLEDState * lhs, const ros_robot_controller_msgs__msg__OLEDState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // index
  if (lhs->index != rhs->index) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->text), &(rhs->text)))
  {
    return false;
  }
  return true;
}

bool
ros_robot_controller_msgs__msg__OLEDState__copy(
  const ros_robot_controller_msgs__msg__OLEDState * input,
  ros_robot_controller_msgs__msg__OLEDState * output)
{
  if (!input || !output) {
    return false;
  }
  // index
  output->index = input->index;
  // text
  if (!rosidl_runtime_c__String__copy(
      &(input->text), &(output->text)))
  {
    return false;
  }
  return true;
}

ros_robot_controller_msgs__msg__OLEDState *
ros_robot_controller_msgs__msg__OLEDState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__OLEDState * msg = (ros_robot_controller_msgs__msg__OLEDState *)allocator.allocate(sizeof(ros_robot_controller_msgs__msg__OLEDState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_robot_controller_msgs__msg__OLEDState));
  bool success = ros_robot_controller_msgs__msg__OLEDState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_robot_controller_msgs__msg__OLEDState__destroy(ros_robot_controller_msgs__msg__OLEDState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_robot_controller_msgs__msg__OLEDState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_robot_controller_msgs__msg__OLEDState__Sequence__init(ros_robot_controller_msgs__msg__OLEDState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__OLEDState * data = NULL;

  if (size) {
    data = (ros_robot_controller_msgs__msg__OLEDState *)allocator.zero_allocate(size, sizeof(ros_robot_controller_msgs__msg__OLEDState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_robot_controller_msgs__msg__OLEDState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_robot_controller_msgs__msg__OLEDState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ros_robot_controller_msgs__msg__OLEDState__Sequence__fini(ros_robot_controller_msgs__msg__OLEDState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ros_robot_controller_msgs__msg__OLEDState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ros_robot_controller_msgs__msg__OLEDState__Sequence *
ros_robot_controller_msgs__msg__OLEDState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__OLEDState__Sequence * array = (ros_robot_controller_msgs__msg__OLEDState__Sequence *)allocator.allocate(sizeof(ros_robot_controller_msgs__msg__OLEDState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_robot_controller_msgs__msg__OLEDState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_robot_controller_msgs__msg__OLEDState__Sequence__destroy(ros_robot_controller_msgs__msg__OLEDState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_robot_controller_msgs__msg__OLEDState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_robot_controller_msgs__msg__OLEDState__Sequence__are_equal(const ros_robot_controller_msgs__msg__OLEDState__Sequence * lhs, const ros_robot_controller_msgs__msg__OLEDState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_robot_controller_msgs__msg__OLEDState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_robot_controller_msgs__msg__OLEDState__Sequence__copy(
  const ros_robot_controller_msgs__msg__OLEDState__Sequence * input,
  ros_robot_controller_msgs__msg__OLEDState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_robot_controller_msgs__msg__OLEDState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_robot_controller_msgs__msg__OLEDState * data =
      (ros_robot_controller_msgs__msg__OLEDState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_robot_controller_msgs__msg__OLEDState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_robot_controller_msgs__msg__OLEDState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_robot_controller_msgs__msg__OLEDState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
