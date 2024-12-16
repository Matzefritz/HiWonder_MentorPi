// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_robot_controller_msgs:msg/ServosPosition.idl
// generated code does not contain a copyright notice
#include "ros_robot_controller_msgs/msg/detail/servos_position__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `position`
#include "ros_robot_controller_msgs/msg/detail/servo_position__functions.h"

bool
ros_robot_controller_msgs__msg__ServosPosition__init(ros_robot_controller_msgs__msg__ServosPosition * msg)
{
  if (!msg) {
    return false;
  }
  // duration
  // position
  if (!ros_robot_controller_msgs__msg__ServoPosition__Sequence__init(&msg->position, 0)) {
    ros_robot_controller_msgs__msg__ServosPosition__fini(msg);
    return false;
  }
  return true;
}

void
ros_robot_controller_msgs__msg__ServosPosition__fini(ros_robot_controller_msgs__msg__ServosPosition * msg)
{
  if (!msg) {
    return;
  }
  // duration
  // position
  ros_robot_controller_msgs__msg__ServoPosition__Sequence__fini(&msg->position);
}

bool
ros_robot_controller_msgs__msg__ServosPosition__are_equal(const ros_robot_controller_msgs__msg__ServosPosition * lhs, const ros_robot_controller_msgs__msg__ServosPosition * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // duration
  if (lhs->duration != rhs->duration) {
    return false;
  }
  // position
  if (!ros_robot_controller_msgs__msg__ServoPosition__Sequence__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  return true;
}

bool
ros_robot_controller_msgs__msg__ServosPosition__copy(
  const ros_robot_controller_msgs__msg__ServosPosition * input,
  ros_robot_controller_msgs__msg__ServosPosition * output)
{
  if (!input || !output) {
    return false;
  }
  // duration
  output->duration = input->duration;
  // position
  if (!ros_robot_controller_msgs__msg__ServoPosition__Sequence__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  return true;
}

ros_robot_controller_msgs__msg__ServosPosition *
ros_robot_controller_msgs__msg__ServosPosition__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__ServosPosition * msg = (ros_robot_controller_msgs__msg__ServosPosition *)allocator.allocate(sizeof(ros_robot_controller_msgs__msg__ServosPosition), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_robot_controller_msgs__msg__ServosPosition));
  bool success = ros_robot_controller_msgs__msg__ServosPosition__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_robot_controller_msgs__msg__ServosPosition__destroy(ros_robot_controller_msgs__msg__ServosPosition * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_robot_controller_msgs__msg__ServosPosition__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_robot_controller_msgs__msg__ServosPosition__Sequence__init(ros_robot_controller_msgs__msg__ServosPosition__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__ServosPosition * data = NULL;

  if (size) {
    data = (ros_robot_controller_msgs__msg__ServosPosition *)allocator.zero_allocate(size, sizeof(ros_robot_controller_msgs__msg__ServosPosition), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_robot_controller_msgs__msg__ServosPosition__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_robot_controller_msgs__msg__ServosPosition__fini(&data[i - 1]);
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
ros_robot_controller_msgs__msg__ServosPosition__Sequence__fini(ros_robot_controller_msgs__msg__ServosPosition__Sequence * array)
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
      ros_robot_controller_msgs__msg__ServosPosition__fini(&array->data[i]);
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

ros_robot_controller_msgs__msg__ServosPosition__Sequence *
ros_robot_controller_msgs__msg__ServosPosition__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__ServosPosition__Sequence * array = (ros_robot_controller_msgs__msg__ServosPosition__Sequence *)allocator.allocate(sizeof(ros_robot_controller_msgs__msg__ServosPosition__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_robot_controller_msgs__msg__ServosPosition__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_robot_controller_msgs__msg__ServosPosition__Sequence__destroy(ros_robot_controller_msgs__msg__ServosPosition__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_robot_controller_msgs__msg__ServosPosition__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_robot_controller_msgs__msg__ServosPosition__Sequence__are_equal(const ros_robot_controller_msgs__msg__ServosPosition__Sequence * lhs, const ros_robot_controller_msgs__msg__ServosPosition__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_robot_controller_msgs__msg__ServosPosition__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_robot_controller_msgs__msg__ServosPosition__Sequence__copy(
  const ros_robot_controller_msgs__msg__ServosPosition__Sequence * input,
  ros_robot_controller_msgs__msg__ServosPosition__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_robot_controller_msgs__msg__ServosPosition);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_robot_controller_msgs__msg__ServosPosition * data =
      (ros_robot_controller_msgs__msg__ServosPosition *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_robot_controller_msgs__msg__ServosPosition__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_robot_controller_msgs__msg__ServosPosition__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_robot_controller_msgs__msg__ServosPosition__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
