// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_robot_controller_msgs:msg/BusServoState.idl
// generated code does not contain a copyright notice
#include "ros_robot_controller_msgs/msg/detail/bus_servo_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `present_id`
// Member `target_id`
// Member `position`
// Member `offset`
// Member `voltage`
// Member `temperature`
// Member `position_limit`
// Member `voltage_limit`
// Member `max_temperature_limit`
// Member `enable_torque`
// Member `save_offset`
// Member `stop`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ros_robot_controller_msgs__msg__BusServoState__init(ros_robot_controller_msgs__msg__BusServoState * msg)
{
  if (!msg) {
    return false;
  }
  // present_id
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->present_id, 0)) {
    ros_robot_controller_msgs__msg__BusServoState__fini(msg);
    return false;
  }
  // target_id
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->target_id, 0)) {
    ros_robot_controller_msgs__msg__BusServoState__fini(msg);
    return false;
  }
  // position
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->position, 0)) {
    ros_robot_controller_msgs__msg__BusServoState__fini(msg);
    return false;
  }
  // offset
  if (!rosidl_runtime_c__int16__Sequence__init(&msg->offset, 0)) {
    ros_robot_controller_msgs__msg__BusServoState__fini(msg);
    return false;
  }
  // voltage
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->voltage, 0)) {
    ros_robot_controller_msgs__msg__BusServoState__fini(msg);
    return false;
  }
  // temperature
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->temperature, 0)) {
    ros_robot_controller_msgs__msg__BusServoState__fini(msg);
    return false;
  }
  // position_limit
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->position_limit, 0)) {
    ros_robot_controller_msgs__msg__BusServoState__fini(msg);
    return false;
  }
  // voltage_limit
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->voltage_limit, 0)) {
    ros_robot_controller_msgs__msg__BusServoState__fini(msg);
    return false;
  }
  // max_temperature_limit
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->max_temperature_limit, 0)) {
    ros_robot_controller_msgs__msg__BusServoState__fini(msg);
    return false;
  }
  // enable_torque
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->enable_torque, 0)) {
    ros_robot_controller_msgs__msg__BusServoState__fini(msg);
    return false;
  }
  // save_offset
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->save_offset, 0)) {
    ros_robot_controller_msgs__msg__BusServoState__fini(msg);
    return false;
  }
  // stop
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->stop, 0)) {
    ros_robot_controller_msgs__msg__BusServoState__fini(msg);
    return false;
  }
  return true;
}

void
ros_robot_controller_msgs__msg__BusServoState__fini(ros_robot_controller_msgs__msg__BusServoState * msg)
{
  if (!msg) {
    return;
  }
  // present_id
  rosidl_runtime_c__uint16__Sequence__fini(&msg->present_id);
  // target_id
  rosidl_runtime_c__uint16__Sequence__fini(&msg->target_id);
  // position
  rosidl_runtime_c__uint16__Sequence__fini(&msg->position);
  // offset
  rosidl_runtime_c__int16__Sequence__fini(&msg->offset);
  // voltage
  rosidl_runtime_c__uint16__Sequence__fini(&msg->voltage);
  // temperature
  rosidl_runtime_c__uint16__Sequence__fini(&msg->temperature);
  // position_limit
  rosidl_runtime_c__uint16__Sequence__fini(&msg->position_limit);
  // voltage_limit
  rosidl_runtime_c__uint16__Sequence__fini(&msg->voltage_limit);
  // max_temperature_limit
  rosidl_runtime_c__uint16__Sequence__fini(&msg->max_temperature_limit);
  // enable_torque
  rosidl_runtime_c__uint16__Sequence__fini(&msg->enable_torque);
  // save_offset
  rosidl_runtime_c__uint16__Sequence__fini(&msg->save_offset);
  // stop
  rosidl_runtime_c__uint16__Sequence__fini(&msg->stop);
}

bool
ros_robot_controller_msgs__msg__BusServoState__are_equal(const ros_robot_controller_msgs__msg__BusServoState * lhs, const ros_robot_controller_msgs__msg__BusServoState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // present_id
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->present_id), &(rhs->present_id)))
  {
    return false;
  }
  // target_id
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->target_id), &(rhs->target_id)))
  {
    return false;
  }
  // position
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // offset
  if (!rosidl_runtime_c__int16__Sequence__are_equal(
      &(lhs->offset), &(rhs->offset)))
  {
    return false;
  }
  // voltage
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->voltage), &(rhs->voltage)))
  {
    return false;
  }
  // temperature
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->temperature), &(rhs->temperature)))
  {
    return false;
  }
  // position_limit
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->position_limit), &(rhs->position_limit)))
  {
    return false;
  }
  // voltage_limit
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->voltage_limit), &(rhs->voltage_limit)))
  {
    return false;
  }
  // max_temperature_limit
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->max_temperature_limit), &(rhs->max_temperature_limit)))
  {
    return false;
  }
  // enable_torque
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->enable_torque), &(rhs->enable_torque)))
  {
    return false;
  }
  // save_offset
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->save_offset), &(rhs->save_offset)))
  {
    return false;
  }
  // stop
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->stop), &(rhs->stop)))
  {
    return false;
  }
  return true;
}

bool
ros_robot_controller_msgs__msg__BusServoState__copy(
  const ros_robot_controller_msgs__msg__BusServoState * input,
  ros_robot_controller_msgs__msg__BusServoState * output)
{
  if (!input || !output) {
    return false;
  }
  // present_id
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->present_id), &(output->present_id)))
  {
    return false;
  }
  // target_id
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->target_id), &(output->target_id)))
  {
    return false;
  }
  // position
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // offset
  if (!rosidl_runtime_c__int16__Sequence__copy(
      &(input->offset), &(output->offset)))
  {
    return false;
  }
  // voltage
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->voltage), &(output->voltage)))
  {
    return false;
  }
  // temperature
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->temperature), &(output->temperature)))
  {
    return false;
  }
  // position_limit
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->position_limit), &(output->position_limit)))
  {
    return false;
  }
  // voltage_limit
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->voltage_limit), &(output->voltage_limit)))
  {
    return false;
  }
  // max_temperature_limit
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->max_temperature_limit), &(output->max_temperature_limit)))
  {
    return false;
  }
  // enable_torque
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->enable_torque), &(output->enable_torque)))
  {
    return false;
  }
  // save_offset
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->save_offset), &(output->save_offset)))
  {
    return false;
  }
  // stop
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->stop), &(output->stop)))
  {
    return false;
  }
  return true;
}

ros_robot_controller_msgs__msg__BusServoState *
ros_robot_controller_msgs__msg__BusServoState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__BusServoState * msg = (ros_robot_controller_msgs__msg__BusServoState *)allocator.allocate(sizeof(ros_robot_controller_msgs__msg__BusServoState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_robot_controller_msgs__msg__BusServoState));
  bool success = ros_robot_controller_msgs__msg__BusServoState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_robot_controller_msgs__msg__BusServoState__destroy(ros_robot_controller_msgs__msg__BusServoState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_robot_controller_msgs__msg__BusServoState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_robot_controller_msgs__msg__BusServoState__Sequence__init(ros_robot_controller_msgs__msg__BusServoState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__BusServoState * data = NULL;

  if (size) {
    data = (ros_robot_controller_msgs__msg__BusServoState *)allocator.zero_allocate(size, sizeof(ros_robot_controller_msgs__msg__BusServoState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_robot_controller_msgs__msg__BusServoState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_robot_controller_msgs__msg__BusServoState__fini(&data[i - 1]);
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
ros_robot_controller_msgs__msg__BusServoState__Sequence__fini(ros_robot_controller_msgs__msg__BusServoState__Sequence * array)
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
      ros_robot_controller_msgs__msg__BusServoState__fini(&array->data[i]);
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

ros_robot_controller_msgs__msg__BusServoState__Sequence *
ros_robot_controller_msgs__msg__BusServoState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__BusServoState__Sequence * array = (ros_robot_controller_msgs__msg__BusServoState__Sequence *)allocator.allocate(sizeof(ros_robot_controller_msgs__msg__BusServoState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_robot_controller_msgs__msg__BusServoState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_robot_controller_msgs__msg__BusServoState__Sequence__destroy(ros_robot_controller_msgs__msg__BusServoState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_robot_controller_msgs__msg__BusServoState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_robot_controller_msgs__msg__BusServoState__Sequence__are_equal(const ros_robot_controller_msgs__msg__BusServoState__Sequence * lhs, const ros_robot_controller_msgs__msg__BusServoState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_robot_controller_msgs__msg__BusServoState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_robot_controller_msgs__msg__BusServoState__Sequence__copy(
  const ros_robot_controller_msgs__msg__BusServoState__Sequence * input,
  ros_robot_controller_msgs__msg__BusServoState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_robot_controller_msgs__msg__BusServoState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_robot_controller_msgs__msg__BusServoState * data =
      (ros_robot_controller_msgs__msg__BusServoState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_robot_controller_msgs__msg__BusServoState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_robot_controller_msgs__msg__BusServoState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_robot_controller_msgs__msg__BusServoState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
