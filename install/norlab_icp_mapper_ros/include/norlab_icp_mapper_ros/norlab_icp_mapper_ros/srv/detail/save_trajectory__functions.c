// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from norlab_icp_mapper_ros:srv/SaveTrajectory.idl
// generated code does not contain a copyright notice
#include "norlab_icp_mapper_ros/srv/detail/save_trajectory__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `trajectory_file_name`
#include "std_msgs/msg/detail/string__functions.h"

bool
norlab_icp_mapper_ros__srv__SaveTrajectory_Request__init(norlab_icp_mapper_ros__srv__SaveTrajectory_Request * msg)
{
  if (!msg) {
    return false;
  }
  // trajectory_file_name
  if (!std_msgs__msg__String__init(&msg->trajectory_file_name)) {
    norlab_icp_mapper_ros__srv__SaveTrajectory_Request__fini(msg);
    return false;
  }
  return true;
}

void
norlab_icp_mapper_ros__srv__SaveTrajectory_Request__fini(norlab_icp_mapper_ros__srv__SaveTrajectory_Request * msg)
{
  if (!msg) {
    return;
  }
  // trajectory_file_name
  std_msgs__msg__String__fini(&msg->trajectory_file_name);
}

bool
norlab_icp_mapper_ros__srv__SaveTrajectory_Request__are_equal(const norlab_icp_mapper_ros__srv__SaveTrajectory_Request * lhs, const norlab_icp_mapper_ros__srv__SaveTrajectory_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // trajectory_file_name
  if (!std_msgs__msg__String__are_equal(
      &(lhs->trajectory_file_name), &(rhs->trajectory_file_name)))
  {
    return false;
  }
  return true;
}

bool
norlab_icp_mapper_ros__srv__SaveTrajectory_Request__copy(
  const norlab_icp_mapper_ros__srv__SaveTrajectory_Request * input,
  norlab_icp_mapper_ros__srv__SaveTrajectory_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // trajectory_file_name
  if (!std_msgs__msg__String__copy(
      &(input->trajectory_file_name), &(output->trajectory_file_name)))
  {
    return false;
  }
  return true;
}

norlab_icp_mapper_ros__srv__SaveTrajectory_Request *
norlab_icp_mapper_ros__srv__SaveTrajectory_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  norlab_icp_mapper_ros__srv__SaveTrajectory_Request * msg = (norlab_icp_mapper_ros__srv__SaveTrajectory_Request *)allocator.allocate(sizeof(norlab_icp_mapper_ros__srv__SaveTrajectory_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(norlab_icp_mapper_ros__srv__SaveTrajectory_Request));
  bool success = norlab_icp_mapper_ros__srv__SaveTrajectory_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
norlab_icp_mapper_ros__srv__SaveTrajectory_Request__destroy(norlab_icp_mapper_ros__srv__SaveTrajectory_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    norlab_icp_mapper_ros__srv__SaveTrajectory_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence__init(norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  norlab_icp_mapper_ros__srv__SaveTrajectory_Request * data = NULL;

  if (size) {
    data = (norlab_icp_mapper_ros__srv__SaveTrajectory_Request *)allocator.zero_allocate(size, sizeof(norlab_icp_mapper_ros__srv__SaveTrajectory_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = norlab_icp_mapper_ros__srv__SaveTrajectory_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        norlab_icp_mapper_ros__srv__SaveTrajectory_Request__fini(&data[i - 1]);
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
norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence__fini(norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence * array)
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
      norlab_icp_mapper_ros__srv__SaveTrajectory_Request__fini(&array->data[i]);
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

norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence *
norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence * array = (norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence *)allocator.allocate(sizeof(norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence__destroy(norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence__are_equal(const norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence * lhs, const norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!norlab_icp_mapper_ros__srv__SaveTrajectory_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence__copy(
  const norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence * input,
  norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(norlab_icp_mapper_ros__srv__SaveTrajectory_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    norlab_icp_mapper_ros__srv__SaveTrajectory_Request * data =
      (norlab_icp_mapper_ros__srv__SaveTrajectory_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!norlab_icp_mapper_ros__srv__SaveTrajectory_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          norlab_icp_mapper_ros__srv__SaveTrajectory_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!norlab_icp_mapper_ros__srv__SaveTrajectory_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
norlab_icp_mapper_ros__srv__SaveTrajectory_Response__init(norlab_icp_mapper_ros__srv__SaveTrajectory_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
norlab_icp_mapper_ros__srv__SaveTrajectory_Response__fini(norlab_icp_mapper_ros__srv__SaveTrajectory_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
norlab_icp_mapper_ros__srv__SaveTrajectory_Response__are_equal(const norlab_icp_mapper_ros__srv__SaveTrajectory_Response * lhs, const norlab_icp_mapper_ros__srv__SaveTrajectory_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
norlab_icp_mapper_ros__srv__SaveTrajectory_Response__copy(
  const norlab_icp_mapper_ros__srv__SaveTrajectory_Response * input,
  norlab_icp_mapper_ros__srv__SaveTrajectory_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

norlab_icp_mapper_ros__srv__SaveTrajectory_Response *
norlab_icp_mapper_ros__srv__SaveTrajectory_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  norlab_icp_mapper_ros__srv__SaveTrajectory_Response * msg = (norlab_icp_mapper_ros__srv__SaveTrajectory_Response *)allocator.allocate(sizeof(norlab_icp_mapper_ros__srv__SaveTrajectory_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(norlab_icp_mapper_ros__srv__SaveTrajectory_Response));
  bool success = norlab_icp_mapper_ros__srv__SaveTrajectory_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
norlab_icp_mapper_ros__srv__SaveTrajectory_Response__destroy(norlab_icp_mapper_ros__srv__SaveTrajectory_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    norlab_icp_mapper_ros__srv__SaveTrajectory_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence__init(norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  norlab_icp_mapper_ros__srv__SaveTrajectory_Response * data = NULL;

  if (size) {
    data = (norlab_icp_mapper_ros__srv__SaveTrajectory_Response *)allocator.zero_allocate(size, sizeof(norlab_icp_mapper_ros__srv__SaveTrajectory_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = norlab_icp_mapper_ros__srv__SaveTrajectory_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        norlab_icp_mapper_ros__srv__SaveTrajectory_Response__fini(&data[i - 1]);
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
norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence__fini(norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence * array)
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
      norlab_icp_mapper_ros__srv__SaveTrajectory_Response__fini(&array->data[i]);
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

norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence *
norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence * array = (norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence *)allocator.allocate(sizeof(norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence__destroy(norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence__are_equal(const norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence * lhs, const norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!norlab_icp_mapper_ros__srv__SaveTrajectory_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence__copy(
  const norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence * input,
  norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(norlab_icp_mapper_ros__srv__SaveTrajectory_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    norlab_icp_mapper_ros__srv__SaveTrajectory_Response * data =
      (norlab_icp_mapper_ros__srv__SaveTrajectory_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!norlab_icp_mapper_ros__srv__SaveTrajectory_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          norlab_icp_mapper_ros__srv__SaveTrajectory_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!norlab_icp_mapper_ros__srv__SaveTrajectory_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
