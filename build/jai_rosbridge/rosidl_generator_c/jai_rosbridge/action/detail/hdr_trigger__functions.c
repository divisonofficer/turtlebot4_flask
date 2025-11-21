// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jai_rosbridge:action/HDRTrigger.idl
// generated code does not contain a copyright notice
#include "jai_rosbridge/action/detail/hdr_trigger__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `space_id`
#include "rosidl_runtime_c/string_functions.h"

bool
jai_rosbridge__action__HDRTrigger_Goal__init(jai_rosbridge__action__HDRTrigger_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // space_id
  if (!rosidl_runtime_c__String__init(&msg->space_id)) {
    jai_rosbridge__action__HDRTrigger_Goal__fini(msg);
    return false;
  }
  return true;
}

void
jai_rosbridge__action__HDRTrigger_Goal__fini(jai_rosbridge__action__HDRTrigger_Goal * msg)
{
  if (!msg) {
    return;
  }
  // space_id
  rosidl_runtime_c__String__fini(&msg->space_id);
}

bool
jai_rosbridge__action__HDRTrigger_Goal__are_equal(const jai_rosbridge__action__HDRTrigger_Goal * lhs, const jai_rosbridge__action__HDRTrigger_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // space_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->space_id), &(rhs->space_id)))
  {
    return false;
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_Goal__copy(
  const jai_rosbridge__action__HDRTrigger_Goal * input,
  jai_rosbridge__action__HDRTrigger_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // space_id
  if (!rosidl_runtime_c__String__copy(
      &(input->space_id), &(output->space_id)))
  {
    return false;
  }
  return true;
}

jai_rosbridge__action__HDRTrigger_Goal *
jai_rosbridge__action__HDRTrigger_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_Goal * msg = (jai_rosbridge__action__HDRTrigger_Goal *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jai_rosbridge__action__HDRTrigger_Goal));
  bool success = jai_rosbridge__action__HDRTrigger_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jai_rosbridge__action__HDRTrigger_Goal__destroy(jai_rosbridge__action__HDRTrigger_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jai_rosbridge__action__HDRTrigger_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jai_rosbridge__action__HDRTrigger_Goal__Sequence__init(jai_rosbridge__action__HDRTrigger_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_Goal * data = NULL;

  if (size) {
    data = (jai_rosbridge__action__HDRTrigger_Goal *)allocator.zero_allocate(size, sizeof(jai_rosbridge__action__HDRTrigger_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jai_rosbridge__action__HDRTrigger_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jai_rosbridge__action__HDRTrigger_Goal__fini(&data[i - 1]);
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
jai_rosbridge__action__HDRTrigger_Goal__Sequence__fini(jai_rosbridge__action__HDRTrigger_Goal__Sequence * array)
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
      jai_rosbridge__action__HDRTrigger_Goal__fini(&array->data[i]);
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

jai_rosbridge__action__HDRTrigger_Goal__Sequence *
jai_rosbridge__action__HDRTrigger_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_Goal__Sequence * array = (jai_rosbridge__action__HDRTrigger_Goal__Sequence *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jai_rosbridge__action__HDRTrigger_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jai_rosbridge__action__HDRTrigger_Goal__Sequence__destroy(jai_rosbridge__action__HDRTrigger_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jai_rosbridge__action__HDRTrigger_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jai_rosbridge__action__HDRTrigger_Goal__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_Goal__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_Goal__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_Goal__Sequence * input,
  jai_rosbridge__action__HDRTrigger_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jai_rosbridge__action__HDRTrigger_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jai_rosbridge__action__HDRTrigger_Goal * data =
      (jai_rosbridge__action__HDRTrigger_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jai_rosbridge__action__HDRTrigger_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jai_rosbridge__action__HDRTrigger_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
jai_rosbridge__action__HDRTrigger_Result__init(jai_rosbridge__action__HDRTrigger_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // result_message
  if (!rosidl_runtime_c__String__init(&msg->result_message)) {
    jai_rosbridge__action__HDRTrigger_Result__fini(msg);
    return false;
  }
  return true;
}

void
jai_rosbridge__action__HDRTrigger_Result__fini(jai_rosbridge__action__HDRTrigger_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // result_message
  rosidl_runtime_c__String__fini(&msg->result_message);
}

bool
jai_rosbridge__action__HDRTrigger_Result__are_equal(const jai_rosbridge__action__HDRTrigger_Result * lhs, const jai_rosbridge__action__HDRTrigger_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // result_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->result_message), &(rhs->result_message)))
  {
    return false;
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_Result__copy(
  const jai_rosbridge__action__HDRTrigger_Result * input,
  jai_rosbridge__action__HDRTrigger_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // result_message
  if (!rosidl_runtime_c__String__copy(
      &(input->result_message), &(output->result_message)))
  {
    return false;
  }
  return true;
}

jai_rosbridge__action__HDRTrigger_Result *
jai_rosbridge__action__HDRTrigger_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_Result * msg = (jai_rosbridge__action__HDRTrigger_Result *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jai_rosbridge__action__HDRTrigger_Result));
  bool success = jai_rosbridge__action__HDRTrigger_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jai_rosbridge__action__HDRTrigger_Result__destroy(jai_rosbridge__action__HDRTrigger_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jai_rosbridge__action__HDRTrigger_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jai_rosbridge__action__HDRTrigger_Result__Sequence__init(jai_rosbridge__action__HDRTrigger_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_Result * data = NULL;

  if (size) {
    data = (jai_rosbridge__action__HDRTrigger_Result *)allocator.zero_allocate(size, sizeof(jai_rosbridge__action__HDRTrigger_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jai_rosbridge__action__HDRTrigger_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jai_rosbridge__action__HDRTrigger_Result__fini(&data[i - 1]);
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
jai_rosbridge__action__HDRTrigger_Result__Sequence__fini(jai_rosbridge__action__HDRTrigger_Result__Sequence * array)
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
      jai_rosbridge__action__HDRTrigger_Result__fini(&array->data[i]);
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

jai_rosbridge__action__HDRTrigger_Result__Sequence *
jai_rosbridge__action__HDRTrigger_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_Result__Sequence * array = (jai_rosbridge__action__HDRTrigger_Result__Sequence *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jai_rosbridge__action__HDRTrigger_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jai_rosbridge__action__HDRTrigger_Result__Sequence__destroy(jai_rosbridge__action__HDRTrigger_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jai_rosbridge__action__HDRTrigger_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jai_rosbridge__action__HDRTrigger_Result__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_Result__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_Result__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_Result__Sequence * input,
  jai_rosbridge__action__HDRTrigger_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jai_rosbridge__action__HDRTrigger_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jai_rosbridge__action__HDRTrigger_Result * data =
      (jai_rosbridge__action__HDRTrigger_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jai_rosbridge__action__HDRTrigger_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jai_rosbridge__action__HDRTrigger_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `feedback_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
jai_rosbridge__action__HDRTrigger_Feedback__init(jai_rosbridge__action__HDRTrigger_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // feedback_message
  if (!rosidl_runtime_c__String__init(&msg->feedback_message)) {
    jai_rosbridge__action__HDRTrigger_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
jai_rosbridge__action__HDRTrigger_Feedback__fini(jai_rosbridge__action__HDRTrigger_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // feedback_message
  rosidl_runtime_c__String__fini(&msg->feedback_message);
}

bool
jai_rosbridge__action__HDRTrigger_Feedback__are_equal(const jai_rosbridge__action__HDRTrigger_Feedback * lhs, const jai_rosbridge__action__HDRTrigger_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // feedback_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->feedback_message), &(rhs->feedback_message)))
  {
    return false;
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_Feedback__copy(
  const jai_rosbridge__action__HDRTrigger_Feedback * input,
  jai_rosbridge__action__HDRTrigger_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // feedback_message
  if (!rosidl_runtime_c__String__copy(
      &(input->feedback_message), &(output->feedback_message)))
  {
    return false;
  }
  return true;
}

jai_rosbridge__action__HDRTrigger_Feedback *
jai_rosbridge__action__HDRTrigger_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_Feedback * msg = (jai_rosbridge__action__HDRTrigger_Feedback *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jai_rosbridge__action__HDRTrigger_Feedback));
  bool success = jai_rosbridge__action__HDRTrigger_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jai_rosbridge__action__HDRTrigger_Feedback__destroy(jai_rosbridge__action__HDRTrigger_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jai_rosbridge__action__HDRTrigger_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jai_rosbridge__action__HDRTrigger_Feedback__Sequence__init(jai_rosbridge__action__HDRTrigger_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_Feedback * data = NULL;

  if (size) {
    data = (jai_rosbridge__action__HDRTrigger_Feedback *)allocator.zero_allocate(size, sizeof(jai_rosbridge__action__HDRTrigger_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jai_rosbridge__action__HDRTrigger_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jai_rosbridge__action__HDRTrigger_Feedback__fini(&data[i - 1]);
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
jai_rosbridge__action__HDRTrigger_Feedback__Sequence__fini(jai_rosbridge__action__HDRTrigger_Feedback__Sequence * array)
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
      jai_rosbridge__action__HDRTrigger_Feedback__fini(&array->data[i]);
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

jai_rosbridge__action__HDRTrigger_Feedback__Sequence *
jai_rosbridge__action__HDRTrigger_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_Feedback__Sequence * array = (jai_rosbridge__action__HDRTrigger_Feedback__Sequence *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jai_rosbridge__action__HDRTrigger_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jai_rosbridge__action__HDRTrigger_Feedback__Sequence__destroy(jai_rosbridge__action__HDRTrigger_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jai_rosbridge__action__HDRTrigger_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jai_rosbridge__action__HDRTrigger_Feedback__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_Feedback__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_Feedback__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_Feedback__Sequence * input,
  jai_rosbridge__action__HDRTrigger_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jai_rosbridge__action__HDRTrigger_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jai_rosbridge__action__HDRTrigger_Feedback * data =
      (jai_rosbridge__action__HDRTrigger_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jai_rosbridge__action__HDRTrigger_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jai_rosbridge__action__HDRTrigger_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "jai_rosbridge/action/detail/hdr_trigger__functions.h"

bool
jai_rosbridge__action__HDRTrigger_SendGoal_Request__init(jai_rosbridge__action__HDRTrigger_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    jai_rosbridge__action__HDRTrigger_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!jai_rosbridge__action__HDRTrigger_Goal__init(&msg->goal)) {
    jai_rosbridge__action__HDRTrigger_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
jai_rosbridge__action__HDRTrigger_SendGoal_Request__fini(jai_rosbridge__action__HDRTrigger_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  jai_rosbridge__action__HDRTrigger_Goal__fini(&msg->goal);
}

bool
jai_rosbridge__action__HDRTrigger_SendGoal_Request__are_equal(const jai_rosbridge__action__HDRTrigger_SendGoal_Request * lhs, const jai_rosbridge__action__HDRTrigger_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!jai_rosbridge__action__HDRTrigger_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_SendGoal_Request__copy(
  const jai_rosbridge__action__HDRTrigger_SendGoal_Request * input,
  jai_rosbridge__action__HDRTrigger_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!jai_rosbridge__action__HDRTrigger_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

jai_rosbridge__action__HDRTrigger_SendGoal_Request *
jai_rosbridge__action__HDRTrigger_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_SendGoal_Request * msg = (jai_rosbridge__action__HDRTrigger_SendGoal_Request *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jai_rosbridge__action__HDRTrigger_SendGoal_Request));
  bool success = jai_rosbridge__action__HDRTrigger_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jai_rosbridge__action__HDRTrigger_SendGoal_Request__destroy(jai_rosbridge__action__HDRTrigger_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jai_rosbridge__action__HDRTrigger_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__init(jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_SendGoal_Request * data = NULL;

  if (size) {
    data = (jai_rosbridge__action__HDRTrigger_SendGoal_Request *)allocator.zero_allocate(size, sizeof(jai_rosbridge__action__HDRTrigger_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jai_rosbridge__action__HDRTrigger_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jai_rosbridge__action__HDRTrigger_SendGoal_Request__fini(&data[i - 1]);
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
jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__fini(jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * array)
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
      jai_rosbridge__action__HDRTrigger_SendGoal_Request__fini(&array->data[i]);
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

jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence *
jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * array = (jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__destroy(jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * input,
  jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jai_rosbridge__action__HDRTrigger_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jai_rosbridge__action__HDRTrigger_SendGoal_Request * data =
      (jai_rosbridge__action__HDRTrigger_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jai_rosbridge__action__HDRTrigger_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jai_rosbridge__action__HDRTrigger_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
jai_rosbridge__action__HDRTrigger_SendGoal_Response__init(jai_rosbridge__action__HDRTrigger_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    jai_rosbridge__action__HDRTrigger_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
jai_rosbridge__action__HDRTrigger_SendGoal_Response__fini(jai_rosbridge__action__HDRTrigger_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
jai_rosbridge__action__HDRTrigger_SendGoal_Response__are_equal(const jai_rosbridge__action__HDRTrigger_SendGoal_Response * lhs, const jai_rosbridge__action__HDRTrigger_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_SendGoal_Response__copy(
  const jai_rosbridge__action__HDRTrigger_SendGoal_Response * input,
  jai_rosbridge__action__HDRTrigger_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

jai_rosbridge__action__HDRTrigger_SendGoal_Response *
jai_rosbridge__action__HDRTrigger_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_SendGoal_Response * msg = (jai_rosbridge__action__HDRTrigger_SendGoal_Response *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jai_rosbridge__action__HDRTrigger_SendGoal_Response));
  bool success = jai_rosbridge__action__HDRTrigger_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jai_rosbridge__action__HDRTrigger_SendGoal_Response__destroy(jai_rosbridge__action__HDRTrigger_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jai_rosbridge__action__HDRTrigger_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__init(jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_SendGoal_Response * data = NULL;

  if (size) {
    data = (jai_rosbridge__action__HDRTrigger_SendGoal_Response *)allocator.zero_allocate(size, sizeof(jai_rosbridge__action__HDRTrigger_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jai_rosbridge__action__HDRTrigger_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jai_rosbridge__action__HDRTrigger_SendGoal_Response__fini(&data[i - 1]);
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
jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__fini(jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * array)
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
      jai_rosbridge__action__HDRTrigger_SendGoal_Response__fini(&array->data[i]);
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

jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence *
jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * array = (jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__destroy(jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * input,
  jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jai_rosbridge__action__HDRTrigger_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jai_rosbridge__action__HDRTrigger_SendGoal_Response * data =
      (jai_rosbridge__action__HDRTrigger_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jai_rosbridge__action__HDRTrigger_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jai_rosbridge__action__HDRTrigger_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
jai_rosbridge__action__HDRTrigger_GetResult_Request__init(jai_rosbridge__action__HDRTrigger_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    jai_rosbridge__action__HDRTrigger_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
jai_rosbridge__action__HDRTrigger_GetResult_Request__fini(jai_rosbridge__action__HDRTrigger_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
jai_rosbridge__action__HDRTrigger_GetResult_Request__are_equal(const jai_rosbridge__action__HDRTrigger_GetResult_Request * lhs, const jai_rosbridge__action__HDRTrigger_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_GetResult_Request__copy(
  const jai_rosbridge__action__HDRTrigger_GetResult_Request * input,
  jai_rosbridge__action__HDRTrigger_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

jai_rosbridge__action__HDRTrigger_GetResult_Request *
jai_rosbridge__action__HDRTrigger_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_GetResult_Request * msg = (jai_rosbridge__action__HDRTrigger_GetResult_Request *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jai_rosbridge__action__HDRTrigger_GetResult_Request));
  bool success = jai_rosbridge__action__HDRTrigger_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jai_rosbridge__action__HDRTrigger_GetResult_Request__destroy(jai_rosbridge__action__HDRTrigger_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jai_rosbridge__action__HDRTrigger_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__init(jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_GetResult_Request * data = NULL;

  if (size) {
    data = (jai_rosbridge__action__HDRTrigger_GetResult_Request *)allocator.zero_allocate(size, sizeof(jai_rosbridge__action__HDRTrigger_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jai_rosbridge__action__HDRTrigger_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jai_rosbridge__action__HDRTrigger_GetResult_Request__fini(&data[i - 1]);
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
jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__fini(jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * array)
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
      jai_rosbridge__action__HDRTrigger_GetResult_Request__fini(&array->data[i]);
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

jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence *
jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * array = (jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__destroy(jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * input,
  jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jai_rosbridge__action__HDRTrigger_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jai_rosbridge__action__HDRTrigger_GetResult_Request * data =
      (jai_rosbridge__action__HDRTrigger_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jai_rosbridge__action__HDRTrigger_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jai_rosbridge__action__HDRTrigger_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "jai_rosbridge/action/detail/hdr_trigger__functions.h"

bool
jai_rosbridge__action__HDRTrigger_GetResult_Response__init(jai_rosbridge__action__HDRTrigger_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!jai_rosbridge__action__HDRTrigger_Result__init(&msg->result)) {
    jai_rosbridge__action__HDRTrigger_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
jai_rosbridge__action__HDRTrigger_GetResult_Response__fini(jai_rosbridge__action__HDRTrigger_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  jai_rosbridge__action__HDRTrigger_Result__fini(&msg->result);
}

bool
jai_rosbridge__action__HDRTrigger_GetResult_Response__are_equal(const jai_rosbridge__action__HDRTrigger_GetResult_Response * lhs, const jai_rosbridge__action__HDRTrigger_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!jai_rosbridge__action__HDRTrigger_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_GetResult_Response__copy(
  const jai_rosbridge__action__HDRTrigger_GetResult_Response * input,
  jai_rosbridge__action__HDRTrigger_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!jai_rosbridge__action__HDRTrigger_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

jai_rosbridge__action__HDRTrigger_GetResult_Response *
jai_rosbridge__action__HDRTrigger_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_GetResult_Response * msg = (jai_rosbridge__action__HDRTrigger_GetResult_Response *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jai_rosbridge__action__HDRTrigger_GetResult_Response));
  bool success = jai_rosbridge__action__HDRTrigger_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jai_rosbridge__action__HDRTrigger_GetResult_Response__destroy(jai_rosbridge__action__HDRTrigger_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jai_rosbridge__action__HDRTrigger_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__init(jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_GetResult_Response * data = NULL;

  if (size) {
    data = (jai_rosbridge__action__HDRTrigger_GetResult_Response *)allocator.zero_allocate(size, sizeof(jai_rosbridge__action__HDRTrigger_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jai_rosbridge__action__HDRTrigger_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jai_rosbridge__action__HDRTrigger_GetResult_Response__fini(&data[i - 1]);
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
jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__fini(jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * array)
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
      jai_rosbridge__action__HDRTrigger_GetResult_Response__fini(&array->data[i]);
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

jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence *
jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * array = (jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__destroy(jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * input,
  jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jai_rosbridge__action__HDRTrigger_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jai_rosbridge__action__HDRTrigger_GetResult_Response * data =
      (jai_rosbridge__action__HDRTrigger_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jai_rosbridge__action__HDRTrigger_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jai_rosbridge__action__HDRTrigger_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "jai_rosbridge/action/detail/hdr_trigger__functions.h"

bool
jai_rosbridge__action__HDRTrigger_FeedbackMessage__init(jai_rosbridge__action__HDRTrigger_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    jai_rosbridge__action__HDRTrigger_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!jai_rosbridge__action__HDRTrigger_Feedback__init(&msg->feedback)) {
    jai_rosbridge__action__HDRTrigger_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
jai_rosbridge__action__HDRTrigger_FeedbackMessage__fini(jai_rosbridge__action__HDRTrigger_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  jai_rosbridge__action__HDRTrigger_Feedback__fini(&msg->feedback);
}

bool
jai_rosbridge__action__HDRTrigger_FeedbackMessage__are_equal(const jai_rosbridge__action__HDRTrigger_FeedbackMessage * lhs, const jai_rosbridge__action__HDRTrigger_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!jai_rosbridge__action__HDRTrigger_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_FeedbackMessage__copy(
  const jai_rosbridge__action__HDRTrigger_FeedbackMessage * input,
  jai_rosbridge__action__HDRTrigger_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!jai_rosbridge__action__HDRTrigger_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

jai_rosbridge__action__HDRTrigger_FeedbackMessage *
jai_rosbridge__action__HDRTrigger_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_FeedbackMessage * msg = (jai_rosbridge__action__HDRTrigger_FeedbackMessage *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jai_rosbridge__action__HDRTrigger_FeedbackMessage));
  bool success = jai_rosbridge__action__HDRTrigger_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jai_rosbridge__action__HDRTrigger_FeedbackMessage__destroy(jai_rosbridge__action__HDRTrigger_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jai_rosbridge__action__HDRTrigger_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__init(jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_FeedbackMessage * data = NULL;

  if (size) {
    data = (jai_rosbridge__action__HDRTrigger_FeedbackMessage *)allocator.zero_allocate(size, sizeof(jai_rosbridge__action__HDRTrigger_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jai_rosbridge__action__HDRTrigger_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jai_rosbridge__action__HDRTrigger_FeedbackMessage__fini(&data[i - 1]);
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
jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__fini(jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * array)
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
      jai_rosbridge__action__HDRTrigger_FeedbackMessage__fini(&array->data[i]);
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

jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence *
jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * array = (jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence *)allocator.allocate(sizeof(jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__destroy(jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * input,
  jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jai_rosbridge__action__HDRTrigger_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jai_rosbridge__action__HDRTrigger_FeedbackMessage * data =
      (jai_rosbridge__action__HDRTrigger_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jai_rosbridge__action__HDRTrigger_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jai_rosbridge__action__HDRTrigger_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jai_rosbridge__action__HDRTrigger_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
