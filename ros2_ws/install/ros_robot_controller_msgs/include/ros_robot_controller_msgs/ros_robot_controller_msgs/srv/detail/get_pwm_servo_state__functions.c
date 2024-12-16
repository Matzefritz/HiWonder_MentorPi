// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_robot_controller_msgs:srv/GetPWMServoState.idl
// generated code does not contain a copyright notice
#include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `cmd`
#include "ros_robot_controller_msgs/msg/detail/get_pwm_servo_cmd__functions.h"

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Request__init(ros_robot_controller_msgs__srv__GetPWMServoState_Request * msg)
{
  if (!msg) {
    return false;
  }
  // cmd
  if (!ros_robot_controller_msgs__msg__GetPWMServoCmd__Sequence__init(&msg->cmd, 0)) {
    ros_robot_controller_msgs__srv__GetPWMServoState_Request__fini(msg);
    return false;
  }
  return true;
}

void
ros_robot_controller_msgs__srv__GetPWMServoState_Request__fini(ros_robot_controller_msgs__srv__GetPWMServoState_Request * msg)
{
  if (!msg) {
    return;
  }
  // cmd
  ros_robot_controller_msgs__msg__GetPWMServoCmd__Sequence__fini(&msg->cmd);
}

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Request__are_equal(const ros_robot_controller_msgs__srv__GetPWMServoState_Request * lhs, const ros_robot_controller_msgs__srv__GetPWMServoState_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmd
  if (!ros_robot_controller_msgs__msg__GetPWMServoCmd__Sequence__are_equal(
      &(lhs->cmd), &(rhs->cmd)))
  {
    return false;
  }
  return true;
}

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Request__copy(
  const ros_robot_controller_msgs__srv__GetPWMServoState_Request * input,
  ros_robot_controller_msgs__srv__GetPWMServoState_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // cmd
  if (!ros_robot_controller_msgs__msg__GetPWMServoCmd__Sequence__copy(
      &(input->cmd), &(output->cmd)))
  {
    return false;
  }
  return true;
}

ros_robot_controller_msgs__srv__GetPWMServoState_Request *
ros_robot_controller_msgs__srv__GetPWMServoState_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__srv__GetPWMServoState_Request * msg = (ros_robot_controller_msgs__srv__GetPWMServoState_Request *)allocator.allocate(sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Request));
  bool success = ros_robot_controller_msgs__srv__GetPWMServoState_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_robot_controller_msgs__srv__GetPWMServoState_Request__destroy(ros_robot_controller_msgs__srv__GetPWMServoState_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_robot_controller_msgs__srv__GetPWMServoState_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence__init(ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__srv__GetPWMServoState_Request * data = NULL;

  if (size) {
    data = (ros_robot_controller_msgs__srv__GetPWMServoState_Request *)allocator.zero_allocate(size, sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_robot_controller_msgs__srv__GetPWMServoState_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_robot_controller_msgs__srv__GetPWMServoState_Request__fini(&data[i - 1]);
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
ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence__fini(ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence * array)
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
      ros_robot_controller_msgs__srv__GetPWMServoState_Request__fini(&array->data[i]);
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

ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence *
ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence * array = (ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence *)allocator.allocate(sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence__destroy(ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence__are_equal(const ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence * lhs, const ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_robot_controller_msgs__srv__GetPWMServoState_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence__copy(
  const ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence * input,
  ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_robot_controller_msgs__srv__GetPWMServoState_Request * data =
      (ros_robot_controller_msgs__srv__GetPWMServoState_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_robot_controller_msgs__srv__GetPWMServoState_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_robot_controller_msgs__srv__GetPWMServoState_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_robot_controller_msgs__srv__GetPWMServoState_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `state`
#include "ros_robot_controller_msgs/msg/detail/pwm_servo_state__functions.h"

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Response__init(ros_robot_controller_msgs__srv__GetPWMServoState_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // state
  if (!ros_robot_controller_msgs__msg__PWMServoState__Sequence__init(&msg->state, 0)) {
    ros_robot_controller_msgs__srv__GetPWMServoState_Response__fini(msg);
    return false;
  }
  return true;
}

void
ros_robot_controller_msgs__srv__GetPWMServoState_Response__fini(ros_robot_controller_msgs__srv__GetPWMServoState_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // state
  ros_robot_controller_msgs__msg__PWMServoState__Sequence__fini(&msg->state);
}

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Response__are_equal(const ros_robot_controller_msgs__srv__GetPWMServoState_Response * lhs, const ros_robot_controller_msgs__srv__GetPWMServoState_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // state
  if (!ros_robot_controller_msgs__msg__PWMServoState__Sequence__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  return true;
}

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Response__copy(
  const ros_robot_controller_msgs__srv__GetPWMServoState_Response * input,
  ros_robot_controller_msgs__srv__GetPWMServoState_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // state
  if (!ros_robot_controller_msgs__msg__PWMServoState__Sequence__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  return true;
}

ros_robot_controller_msgs__srv__GetPWMServoState_Response *
ros_robot_controller_msgs__srv__GetPWMServoState_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__srv__GetPWMServoState_Response * msg = (ros_robot_controller_msgs__srv__GetPWMServoState_Response *)allocator.allocate(sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Response));
  bool success = ros_robot_controller_msgs__srv__GetPWMServoState_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_robot_controller_msgs__srv__GetPWMServoState_Response__destroy(ros_robot_controller_msgs__srv__GetPWMServoState_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_robot_controller_msgs__srv__GetPWMServoState_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence__init(ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__srv__GetPWMServoState_Response * data = NULL;

  if (size) {
    data = (ros_robot_controller_msgs__srv__GetPWMServoState_Response *)allocator.zero_allocate(size, sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_robot_controller_msgs__srv__GetPWMServoState_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_robot_controller_msgs__srv__GetPWMServoState_Response__fini(&data[i - 1]);
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
ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence__fini(ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence * array)
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
      ros_robot_controller_msgs__srv__GetPWMServoState_Response__fini(&array->data[i]);
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

ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence *
ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence * array = (ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence *)allocator.allocate(sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence__destroy(ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence__are_equal(const ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence * lhs, const ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_robot_controller_msgs__srv__GetPWMServoState_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence__copy(
  const ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence * input,
  ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_robot_controller_msgs__srv__GetPWMServoState_Response * data =
      (ros_robot_controller_msgs__srv__GetPWMServoState_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_robot_controller_msgs__srv__GetPWMServoState_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_robot_controller_msgs__srv__GetPWMServoState_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_robot_controller_msgs__srv__GetPWMServoState_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "ros_robot_controller_msgs/srv/detail/get_pwm_servo_state__functions.h"

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Event__init(ros_robot_controller_msgs__srv__GetPWMServoState_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    ros_robot_controller_msgs__srv__GetPWMServoState_Event__fini(msg);
    return false;
  }
  // request
  if (!ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence__init(&msg->request, 0)) {
    ros_robot_controller_msgs__srv__GetPWMServoState_Event__fini(msg);
    return false;
  }
  // response
  if (!ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence__init(&msg->response, 0)) {
    ros_robot_controller_msgs__srv__GetPWMServoState_Event__fini(msg);
    return false;
  }
  return true;
}

void
ros_robot_controller_msgs__srv__GetPWMServoState_Event__fini(ros_robot_controller_msgs__srv__GetPWMServoState_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence__fini(&msg->request);
  // response
  ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence__fini(&msg->response);
}

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Event__are_equal(const ros_robot_controller_msgs__srv__GetPWMServoState_Event * lhs, const ros_robot_controller_msgs__srv__GetPWMServoState_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Event__copy(
  const ros_robot_controller_msgs__srv__GetPWMServoState_Event * input,
  ros_robot_controller_msgs__srv__GetPWMServoState_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!ros_robot_controller_msgs__srv__GetPWMServoState_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!ros_robot_controller_msgs__srv__GetPWMServoState_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

ros_robot_controller_msgs__srv__GetPWMServoState_Event *
ros_robot_controller_msgs__srv__GetPWMServoState_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__srv__GetPWMServoState_Event * msg = (ros_robot_controller_msgs__srv__GetPWMServoState_Event *)allocator.allocate(sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Event));
  bool success = ros_robot_controller_msgs__srv__GetPWMServoState_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_robot_controller_msgs__srv__GetPWMServoState_Event__destroy(ros_robot_controller_msgs__srv__GetPWMServoState_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_robot_controller_msgs__srv__GetPWMServoState_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence__init(ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__srv__GetPWMServoState_Event * data = NULL;

  if (size) {
    data = (ros_robot_controller_msgs__srv__GetPWMServoState_Event *)allocator.zero_allocate(size, sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_robot_controller_msgs__srv__GetPWMServoState_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_robot_controller_msgs__srv__GetPWMServoState_Event__fini(&data[i - 1]);
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
ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence__fini(ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence * array)
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
      ros_robot_controller_msgs__srv__GetPWMServoState_Event__fini(&array->data[i]);
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

ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence *
ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence * array = (ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence *)allocator.allocate(sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence__destroy(ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence__are_equal(const ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence * lhs, const ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_robot_controller_msgs__srv__GetPWMServoState_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence__copy(
  const ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence * input,
  ros_robot_controller_msgs__srv__GetPWMServoState_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_robot_controller_msgs__srv__GetPWMServoState_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_robot_controller_msgs__srv__GetPWMServoState_Event * data =
      (ros_robot_controller_msgs__srv__GetPWMServoState_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_robot_controller_msgs__srv__GetPWMServoState_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_robot_controller_msgs__srv__GetPWMServoState_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_robot_controller_msgs__srv__GetPWMServoState_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
