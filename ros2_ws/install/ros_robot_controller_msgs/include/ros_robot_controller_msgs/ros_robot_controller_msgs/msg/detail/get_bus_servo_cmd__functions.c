// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_robot_controller_msgs:msg/GetBusServoCmd.idl
// generated code does not contain a copyright notice
#include "ros_robot_controller_msgs/msg/detail/get_bus_servo_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ros_robot_controller_msgs__msg__GetBusServoCmd__init(ros_robot_controller_msgs__msg__GetBusServoCmd * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // get_id
  // get_position
  // get_offset
  // get_voltage
  // get_temperature
  // get_position_limit
  // get_voltage_limit
  // get_max_temperature_limit
  // get_torque_state
  return true;
}

void
ros_robot_controller_msgs__msg__GetBusServoCmd__fini(ros_robot_controller_msgs__msg__GetBusServoCmd * msg)
{
  if (!msg) {
    return;
  }
  // id
  // get_id
  // get_position
  // get_offset
  // get_voltage
  // get_temperature
  // get_position_limit
  // get_voltage_limit
  // get_max_temperature_limit
  // get_torque_state
}

bool
ros_robot_controller_msgs__msg__GetBusServoCmd__are_equal(const ros_robot_controller_msgs__msg__GetBusServoCmd * lhs, const ros_robot_controller_msgs__msg__GetBusServoCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // get_id
  if (lhs->get_id != rhs->get_id) {
    return false;
  }
  // get_position
  if (lhs->get_position != rhs->get_position) {
    return false;
  }
  // get_offset
  if (lhs->get_offset != rhs->get_offset) {
    return false;
  }
  // get_voltage
  if (lhs->get_voltage != rhs->get_voltage) {
    return false;
  }
  // get_temperature
  if (lhs->get_temperature != rhs->get_temperature) {
    return false;
  }
  // get_position_limit
  if (lhs->get_position_limit != rhs->get_position_limit) {
    return false;
  }
  // get_voltage_limit
  if (lhs->get_voltage_limit != rhs->get_voltage_limit) {
    return false;
  }
  // get_max_temperature_limit
  if (lhs->get_max_temperature_limit != rhs->get_max_temperature_limit) {
    return false;
  }
  // get_torque_state
  if (lhs->get_torque_state != rhs->get_torque_state) {
    return false;
  }
  return true;
}

bool
ros_robot_controller_msgs__msg__GetBusServoCmd__copy(
  const ros_robot_controller_msgs__msg__GetBusServoCmd * input,
  ros_robot_controller_msgs__msg__GetBusServoCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // get_id
  output->get_id = input->get_id;
  // get_position
  output->get_position = input->get_position;
  // get_offset
  output->get_offset = input->get_offset;
  // get_voltage
  output->get_voltage = input->get_voltage;
  // get_temperature
  output->get_temperature = input->get_temperature;
  // get_position_limit
  output->get_position_limit = input->get_position_limit;
  // get_voltage_limit
  output->get_voltage_limit = input->get_voltage_limit;
  // get_max_temperature_limit
  output->get_max_temperature_limit = input->get_max_temperature_limit;
  // get_torque_state
  output->get_torque_state = input->get_torque_state;
  return true;
}

ros_robot_controller_msgs__msg__GetBusServoCmd *
ros_robot_controller_msgs__msg__GetBusServoCmd__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__GetBusServoCmd * msg = (ros_robot_controller_msgs__msg__GetBusServoCmd *)allocator.allocate(sizeof(ros_robot_controller_msgs__msg__GetBusServoCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_robot_controller_msgs__msg__GetBusServoCmd));
  bool success = ros_robot_controller_msgs__msg__GetBusServoCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_robot_controller_msgs__msg__GetBusServoCmd__destroy(ros_robot_controller_msgs__msg__GetBusServoCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_robot_controller_msgs__msg__GetBusServoCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence__init(ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__GetBusServoCmd * data = NULL;

  if (size) {
    data = (ros_robot_controller_msgs__msg__GetBusServoCmd *)allocator.zero_allocate(size, sizeof(ros_robot_controller_msgs__msg__GetBusServoCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_robot_controller_msgs__msg__GetBusServoCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_robot_controller_msgs__msg__GetBusServoCmd__fini(&data[i - 1]);
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
ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence__fini(ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence * array)
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
      ros_robot_controller_msgs__msg__GetBusServoCmd__fini(&array->data[i]);
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

ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence *
ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence * array = (ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence *)allocator.allocate(sizeof(ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence__destroy(ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence__are_equal(const ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence * lhs, const ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_robot_controller_msgs__msg__GetBusServoCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence__copy(
  const ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence * input,
  ros_robot_controller_msgs__msg__GetBusServoCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_robot_controller_msgs__msg__GetBusServoCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_robot_controller_msgs__msg__GetBusServoCmd * data =
      (ros_robot_controller_msgs__msg__GetBusServoCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_robot_controller_msgs__msg__GetBusServoCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_robot_controller_msgs__msg__GetBusServoCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_robot_controller_msgs__msg__GetBusServoCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
