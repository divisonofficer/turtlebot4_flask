// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dcs103e_controller:srv/SetChannelMode.idl
// generated code does not contain a copyright notice
#include "dcs103e_controller/srv/detail/set_channel_mode__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `mode`
#include "rosidl_runtime_c/string_functions.h"

bool
dcs103e_controller__srv__SetChannelMode_Request__init(dcs103e_controller__srv__SetChannelMode_Request * msg)
{
  if (!msg) {
    return false;
  }
  // channel
  // mode
  if (!rosidl_runtime_c__String__init(&msg->mode)) {
    dcs103e_controller__srv__SetChannelMode_Request__fini(msg);
    return false;
  }
  // current
  // pulse_width
  // pulse_delay
  return true;
}

void
dcs103e_controller__srv__SetChannelMode_Request__fini(dcs103e_controller__srv__SetChannelMode_Request * msg)
{
  if (!msg) {
    return;
  }
  // channel
  // mode
  rosidl_runtime_c__String__fini(&msg->mode);
  // current
  // pulse_width
  // pulse_delay
}

bool
dcs103e_controller__srv__SetChannelMode_Request__are_equal(const dcs103e_controller__srv__SetChannelMode_Request * lhs, const dcs103e_controller__srv__SetChannelMode_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // channel
  if (lhs->channel != rhs->channel) {
    return false;
  }
  // mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mode), &(rhs->mode)))
  {
    return false;
  }
  // current
  if (lhs->current != rhs->current) {
    return false;
  }
  // pulse_width
  if (lhs->pulse_width != rhs->pulse_width) {
    return false;
  }
  // pulse_delay
  if (lhs->pulse_delay != rhs->pulse_delay) {
    return false;
  }
  return true;
}

bool
dcs103e_controller__srv__SetChannelMode_Request__copy(
  const dcs103e_controller__srv__SetChannelMode_Request * input,
  dcs103e_controller__srv__SetChannelMode_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // channel
  output->channel = input->channel;
  // mode
  if (!rosidl_runtime_c__String__copy(
      &(input->mode), &(output->mode)))
  {
    return false;
  }
  // current
  output->current = input->current;
  // pulse_width
  output->pulse_width = input->pulse_width;
  // pulse_delay
  output->pulse_delay = input->pulse_delay;
  return true;
}

dcs103e_controller__srv__SetChannelMode_Request *
dcs103e_controller__srv__SetChannelMode_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dcs103e_controller__srv__SetChannelMode_Request * msg = (dcs103e_controller__srv__SetChannelMode_Request *)allocator.allocate(sizeof(dcs103e_controller__srv__SetChannelMode_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dcs103e_controller__srv__SetChannelMode_Request));
  bool success = dcs103e_controller__srv__SetChannelMode_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dcs103e_controller__srv__SetChannelMode_Request__destroy(dcs103e_controller__srv__SetChannelMode_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dcs103e_controller__srv__SetChannelMode_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dcs103e_controller__srv__SetChannelMode_Request__Sequence__init(dcs103e_controller__srv__SetChannelMode_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dcs103e_controller__srv__SetChannelMode_Request * data = NULL;

  if (size) {
    data = (dcs103e_controller__srv__SetChannelMode_Request *)allocator.zero_allocate(size, sizeof(dcs103e_controller__srv__SetChannelMode_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dcs103e_controller__srv__SetChannelMode_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dcs103e_controller__srv__SetChannelMode_Request__fini(&data[i - 1]);
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
dcs103e_controller__srv__SetChannelMode_Request__Sequence__fini(dcs103e_controller__srv__SetChannelMode_Request__Sequence * array)
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
      dcs103e_controller__srv__SetChannelMode_Request__fini(&array->data[i]);
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

dcs103e_controller__srv__SetChannelMode_Request__Sequence *
dcs103e_controller__srv__SetChannelMode_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dcs103e_controller__srv__SetChannelMode_Request__Sequence * array = (dcs103e_controller__srv__SetChannelMode_Request__Sequence *)allocator.allocate(sizeof(dcs103e_controller__srv__SetChannelMode_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dcs103e_controller__srv__SetChannelMode_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dcs103e_controller__srv__SetChannelMode_Request__Sequence__destroy(dcs103e_controller__srv__SetChannelMode_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dcs103e_controller__srv__SetChannelMode_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dcs103e_controller__srv__SetChannelMode_Request__Sequence__are_equal(const dcs103e_controller__srv__SetChannelMode_Request__Sequence * lhs, const dcs103e_controller__srv__SetChannelMode_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dcs103e_controller__srv__SetChannelMode_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dcs103e_controller__srv__SetChannelMode_Request__Sequence__copy(
  const dcs103e_controller__srv__SetChannelMode_Request__Sequence * input,
  dcs103e_controller__srv__SetChannelMode_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dcs103e_controller__srv__SetChannelMode_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dcs103e_controller__srv__SetChannelMode_Request * data =
      (dcs103e_controller__srv__SetChannelMode_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dcs103e_controller__srv__SetChannelMode_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dcs103e_controller__srv__SetChannelMode_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dcs103e_controller__srv__SetChannelMode_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
dcs103e_controller__srv__SetChannelMode_Response__init(dcs103e_controller__srv__SetChannelMode_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    dcs103e_controller__srv__SetChannelMode_Response__fini(msg);
    return false;
  }
  return true;
}

void
dcs103e_controller__srv__SetChannelMode_Response__fini(dcs103e_controller__srv__SetChannelMode_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
dcs103e_controller__srv__SetChannelMode_Response__are_equal(const dcs103e_controller__srv__SetChannelMode_Response * lhs, const dcs103e_controller__srv__SetChannelMode_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
dcs103e_controller__srv__SetChannelMode_Response__copy(
  const dcs103e_controller__srv__SetChannelMode_Response * input,
  dcs103e_controller__srv__SetChannelMode_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

dcs103e_controller__srv__SetChannelMode_Response *
dcs103e_controller__srv__SetChannelMode_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dcs103e_controller__srv__SetChannelMode_Response * msg = (dcs103e_controller__srv__SetChannelMode_Response *)allocator.allocate(sizeof(dcs103e_controller__srv__SetChannelMode_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dcs103e_controller__srv__SetChannelMode_Response));
  bool success = dcs103e_controller__srv__SetChannelMode_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dcs103e_controller__srv__SetChannelMode_Response__destroy(dcs103e_controller__srv__SetChannelMode_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dcs103e_controller__srv__SetChannelMode_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dcs103e_controller__srv__SetChannelMode_Response__Sequence__init(dcs103e_controller__srv__SetChannelMode_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dcs103e_controller__srv__SetChannelMode_Response * data = NULL;

  if (size) {
    data = (dcs103e_controller__srv__SetChannelMode_Response *)allocator.zero_allocate(size, sizeof(dcs103e_controller__srv__SetChannelMode_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dcs103e_controller__srv__SetChannelMode_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dcs103e_controller__srv__SetChannelMode_Response__fini(&data[i - 1]);
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
dcs103e_controller__srv__SetChannelMode_Response__Sequence__fini(dcs103e_controller__srv__SetChannelMode_Response__Sequence * array)
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
      dcs103e_controller__srv__SetChannelMode_Response__fini(&array->data[i]);
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

dcs103e_controller__srv__SetChannelMode_Response__Sequence *
dcs103e_controller__srv__SetChannelMode_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dcs103e_controller__srv__SetChannelMode_Response__Sequence * array = (dcs103e_controller__srv__SetChannelMode_Response__Sequence *)allocator.allocate(sizeof(dcs103e_controller__srv__SetChannelMode_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dcs103e_controller__srv__SetChannelMode_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dcs103e_controller__srv__SetChannelMode_Response__Sequence__destroy(dcs103e_controller__srv__SetChannelMode_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dcs103e_controller__srv__SetChannelMode_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dcs103e_controller__srv__SetChannelMode_Response__Sequence__are_equal(const dcs103e_controller__srv__SetChannelMode_Response__Sequence * lhs, const dcs103e_controller__srv__SetChannelMode_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dcs103e_controller__srv__SetChannelMode_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dcs103e_controller__srv__SetChannelMode_Response__Sequence__copy(
  const dcs103e_controller__srv__SetChannelMode_Response__Sequence * input,
  dcs103e_controller__srv__SetChannelMode_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dcs103e_controller__srv__SetChannelMode_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dcs103e_controller__srv__SetChannelMode_Response * data =
      (dcs103e_controller__srv__SetChannelMode_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dcs103e_controller__srv__SetChannelMode_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dcs103e_controller__srv__SetChannelMode_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dcs103e_controller__srv__SetChannelMode_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
