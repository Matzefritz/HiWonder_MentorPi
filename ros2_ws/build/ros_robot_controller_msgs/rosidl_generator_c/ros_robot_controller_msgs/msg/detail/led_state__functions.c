// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_robot_controller_msgs:msg/LedState.idl
// generated code does not contain a copyright notice
#include "ros_robot_controller_msgs/msg/detail/led_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ros_robot_controller_msgs__msg__LedState__init(ros_robot_controller_msgs__msg__LedState * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // on_time
  // off_time
  // repeat
  return true;
}

void
ros_robot_controller_msgs__msg__LedState__fini(ros_robot_controller_msgs__msg__LedState * msg)
{
  if (!msg) {
    return;
  }
  // id
  // on_time
  // off_time
  // repeat
}

bool
ros_robot_controller_msgs__msg__LedState__are_equal(const ros_robot_controller_msgs__msg__LedState * lhs, const ros_robot_controller_msgs__msg__LedState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // on_time
  if (lhs->on_time != rhs->on_time) {
    return false;
  }
  // off_time
  if (lhs->off_time != rhs->off_time) {
    return false;
  }
  // repeat
  if (lhs->repeat != rhs->repeat) {
    return false;
  }
  return true;
}

bool
ros_robot_controller_msgs__msg__LedState__copy(
  const ros_robot_controller_msgs__msg__LedState * input,
  ros_robot_controller_msgs__msg__LedState * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // on_time
  output->on_time = input->on_time;
  // off_time
  output->off_time = input->off_time;
  // repeat
  output->repeat = input->repeat;
  return true;
}

ros_robot_controller_msgs__msg__LedState *
ros_robot_controller_msgs__msg__LedState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__LedState * msg = (ros_robot_controller_msgs__msg__LedState *)allocator.allocate(sizeof(ros_robot_controller_msgs__msg__LedState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_robot_controller_msgs__msg__LedState));
  bool success = ros_robot_controller_msgs__msg__LedState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_robot_controller_msgs__msg__LedState__destroy(ros_robot_controller_msgs__msg__LedState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_robot_controller_msgs__msg__LedState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_robot_controller_msgs__msg__LedState__Sequence__init(ros_robot_controller_msgs__msg__LedState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__LedState * data = NULL;

  if (size) {
    data = (ros_robot_controller_msgs__msg__LedState *)allocator.zero_allocate(size, sizeof(ros_robot_controller_msgs__msg__LedState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_robot_controller_msgs__msg__LedState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_robot_controller_msgs__msg__LedState__fini(&data[i - 1]);
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
ros_robot_controller_msgs__msg__LedState__Sequence__fini(ros_robot_controller_msgs__msg__LedState__Sequence * array)
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
      ros_robot_controller_msgs__msg__LedState__fini(&array->data[i]);
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

ros_robot_controller_msgs__msg__LedState__Sequence *
ros_robot_controller_msgs__msg__LedState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__LedState__Sequence * array = (ros_robot_controller_msgs__msg__LedState__Sequence *)allocator.allocate(sizeof(ros_robot_controller_msgs__msg__LedState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_robot_controller_msgs__msg__LedState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_robot_controller_msgs__msg__LedState__Sequence__destroy(ros_robot_controller_msgs__msg__LedState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_robot_controller_msgs__msg__LedState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_robot_controller_msgs__msg__LedState__Sequence__are_equal(const ros_robot_controller_msgs__msg__LedState__Sequence * lhs, const ros_robot_controller_msgs__msg__LedState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_robot_controller_msgs__msg__LedState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_robot_controller_msgs__msg__LedState__Sequence__copy(
  const ros_robot_controller_msgs__msg__LedState__Sequence * input,
  ros_robot_controller_msgs__msg__LedState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_robot_controller_msgs__msg__LedState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_robot_controller_msgs__msg__LedState * data =
      (ros_robot_controller_msgs__msg__LedState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_robot_controller_msgs__msg__LedState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_robot_controller_msgs__msg__LedState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_robot_controller_msgs__msg__LedState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
