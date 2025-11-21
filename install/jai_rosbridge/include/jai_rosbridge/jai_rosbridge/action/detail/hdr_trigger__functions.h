// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from jai_rosbridge:action/HDRTrigger.idl
// generated code does not contain a copyright notice

#ifndef JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__FUNCTIONS_H_
#define JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "jai_rosbridge/msg/rosidl_generator_c__visibility_control.h"

#include "jai_rosbridge/action/detail/hdr_trigger__struct.h"

/// Initialize action/HDRTrigger message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * jai_rosbridge__action__HDRTrigger_Goal
 * )) before or use
 * jai_rosbridge__action__HDRTrigger_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Goal__init(jai_rosbridge__action__HDRTrigger_Goal * msg);

/// Finalize action/HDRTrigger message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_Goal__fini(jai_rosbridge__action__HDRTrigger_Goal * msg);

/// Create action/HDRTrigger message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * jai_rosbridge__action__HDRTrigger_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_Goal *
jai_rosbridge__action__HDRTrigger_Goal__create();

/// Destroy action/HDRTrigger message.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_Goal__destroy(jai_rosbridge__action__HDRTrigger_Goal * msg);

/// Check for action/HDRTrigger message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Goal__are_equal(const jai_rosbridge__action__HDRTrigger_Goal * lhs, const jai_rosbridge__action__HDRTrigger_Goal * rhs);

/// Copy a action/HDRTrigger message.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Goal__copy(
  const jai_rosbridge__action__HDRTrigger_Goal * input,
  jai_rosbridge__action__HDRTrigger_Goal * output);

/// Initialize array of action/HDRTrigger messages.
/**
 * It allocates the memory for the number of elements and calls
 * jai_rosbridge__action__HDRTrigger_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Goal__Sequence__init(jai_rosbridge__action__HDRTrigger_Goal__Sequence * array, size_t size);

/// Finalize array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_Goal__Sequence__fini(jai_rosbridge__action__HDRTrigger_Goal__Sequence * array);

/// Create array of action/HDRTrigger messages.
/**
 * It allocates the memory for the array and calls
 * jai_rosbridge__action__HDRTrigger_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_Goal__Sequence *
jai_rosbridge__action__HDRTrigger_Goal__Sequence__create(size_t size);

/// Destroy array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_Goal__Sequence__destroy(jai_rosbridge__action__HDRTrigger_Goal__Sequence * array);

/// Check for action/HDRTrigger message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Goal__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_Goal__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_Goal__Sequence * rhs);

/// Copy an array of action/HDRTrigger messages.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Goal__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_Goal__Sequence * input,
  jai_rosbridge__action__HDRTrigger_Goal__Sequence * output);

/// Initialize action/HDRTrigger message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * jai_rosbridge__action__HDRTrigger_Result
 * )) before or use
 * jai_rosbridge__action__HDRTrigger_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Result__init(jai_rosbridge__action__HDRTrigger_Result * msg);

/// Finalize action/HDRTrigger message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_Result__fini(jai_rosbridge__action__HDRTrigger_Result * msg);

/// Create action/HDRTrigger message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * jai_rosbridge__action__HDRTrigger_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_Result *
jai_rosbridge__action__HDRTrigger_Result__create();

/// Destroy action/HDRTrigger message.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_Result__destroy(jai_rosbridge__action__HDRTrigger_Result * msg);

/// Check for action/HDRTrigger message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Result__are_equal(const jai_rosbridge__action__HDRTrigger_Result * lhs, const jai_rosbridge__action__HDRTrigger_Result * rhs);

/// Copy a action/HDRTrigger message.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Result__copy(
  const jai_rosbridge__action__HDRTrigger_Result * input,
  jai_rosbridge__action__HDRTrigger_Result * output);

/// Initialize array of action/HDRTrigger messages.
/**
 * It allocates the memory for the number of elements and calls
 * jai_rosbridge__action__HDRTrigger_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Result__Sequence__init(jai_rosbridge__action__HDRTrigger_Result__Sequence * array, size_t size);

/// Finalize array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_Result__Sequence__fini(jai_rosbridge__action__HDRTrigger_Result__Sequence * array);

/// Create array of action/HDRTrigger messages.
/**
 * It allocates the memory for the array and calls
 * jai_rosbridge__action__HDRTrigger_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_Result__Sequence *
jai_rosbridge__action__HDRTrigger_Result__Sequence__create(size_t size);

/// Destroy array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_Result__Sequence__destroy(jai_rosbridge__action__HDRTrigger_Result__Sequence * array);

/// Check for action/HDRTrigger message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Result__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_Result__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_Result__Sequence * rhs);

/// Copy an array of action/HDRTrigger messages.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Result__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_Result__Sequence * input,
  jai_rosbridge__action__HDRTrigger_Result__Sequence * output);

/// Initialize action/HDRTrigger message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * jai_rosbridge__action__HDRTrigger_Feedback
 * )) before or use
 * jai_rosbridge__action__HDRTrigger_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Feedback__init(jai_rosbridge__action__HDRTrigger_Feedback * msg);

/// Finalize action/HDRTrigger message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_Feedback__fini(jai_rosbridge__action__HDRTrigger_Feedback * msg);

/// Create action/HDRTrigger message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * jai_rosbridge__action__HDRTrigger_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_Feedback *
jai_rosbridge__action__HDRTrigger_Feedback__create();

/// Destroy action/HDRTrigger message.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_Feedback__destroy(jai_rosbridge__action__HDRTrigger_Feedback * msg);

/// Check for action/HDRTrigger message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Feedback__are_equal(const jai_rosbridge__action__HDRTrigger_Feedback * lhs, const jai_rosbridge__action__HDRTrigger_Feedback * rhs);

/// Copy a action/HDRTrigger message.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Feedback__copy(
  const jai_rosbridge__action__HDRTrigger_Feedback * input,
  jai_rosbridge__action__HDRTrigger_Feedback * output);

/// Initialize array of action/HDRTrigger messages.
/**
 * It allocates the memory for the number of elements and calls
 * jai_rosbridge__action__HDRTrigger_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Feedback__Sequence__init(jai_rosbridge__action__HDRTrigger_Feedback__Sequence * array, size_t size);

/// Finalize array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_Feedback__Sequence__fini(jai_rosbridge__action__HDRTrigger_Feedback__Sequence * array);

/// Create array of action/HDRTrigger messages.
/**
 * It allocates the memory for the array and calls
 * jai_rosbridge__action__HDRTrigger_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_Feedback__Sequence *
jai_rosbridge__action__HDRTrigger_Feedback__Sequence__create(size_t size);

/// Destroy array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_Feedback__Sequence__destroy(jai_rosbridge__action__HDRTrigger_Feedback__Sequence * array);

/// Check for action/HDRTrigger message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Feedback__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_Feedback__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_Feedback__Sequence * rhs);

/// Copy an array of action/HDRTrigger messages.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_Feedback__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_Feedback__Sequence * input,
  jai_rosbridge__action__HDRTrigger_Feedback__Sequence * output);

/// Initialize action/HDRTrigger message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * jai_rosbridge__action__HDRTrigger_SendGoal_Request
 * )) before or use
 * jai_rosbridge__action__HDRTrigger_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_SendGoal_Request__init(jai_rosbridge__action__HDRTrigger_SendGoal_Request * msg);

/// Finalize action/HDRTrigger message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_SendGoal_Request__fini(jai_rosbridge__action__HDRTrigger_SendGoal_Request * msg);

/// Create action/HDRTrigger message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * jai_rosbridge__action__HDRTrigger_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_SendGoal_Request *
jai_rosbridge__action__HDRTrigger_SendGoal_Request__create();

/// Destroy action/HDRTrigger message.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_SendGoal_Request__destroy(jai_rosbridge__action__HDRTrigger_SendGoal_Request * msg);

/// Check for action/HDRTrigger message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_SendGoal_Request__are_equal(const jai_rosbridge__action__HDRTrigger_SendGoal_Request * lhs, const jai_rosbridge__action__HDRTrigger_SendGoal_Request * rhs);

/// Copy a action/HDRTrigger message.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_SendGoal_Request__copy(
  const jai_rosbridge__action__HDRTrigger_SendGoal_Request * input,
  jai_rosbridge__action__HDRTrigger_SendGoal_Request * output);

/// Initialize array of action/HDRTrigger messages.
/**
 * It allocates the memory for the number of elements and calls
 * jai_rosbridge__action__HDRTrigger_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__init(jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__fini(jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * array);

/// Create array of action/HDRTrigger messages.
/**
 * It allocates the memory for the array and calls
 * jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence *
jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__destroy(jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * array);

/// Check for action/HDRTrigger message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/HDRTrigger messages.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * input,
  jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence * output);

/// Initialize action/HDRTrigger message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * jai_rosbridge__action__HDRTrigger_SendGoal_Response
 * )) before or use
 * jai_rosbridge__action__HDRTrigger_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_SendGoal_Response__init(jai_rosbridge__action__HDRTrigger_SendGoal_Response * msg);

/// Finalize action/HDRTrigger message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_SendGoal_Response__fini(jai_rosbridge__action__HDRTrigger_SendGoal_Response * msg);

/// Create action/HDRTrigger message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * jai_rosbridge__action__HDRTrigger_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_SendGoal_Response *
jai_rosbridge__action__HDRTrigger_SendGoal_Response__create();

/// Destroy action/HDRTrigger message.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_SendGoal_Response__destroy(jai_rosbridge__action__HDRTrigger_SendGoal_Response * msg);

/// Check for action/HDRTrigger message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_SendGoal_Response__are_equal(const jai_rosbridge__action__HDRTrigger_SendGoal_Response * lhs, const jai_rosbridge__action__HDRTrigger_SendGoal_Response * rhs);

/// Copy a action/HDRTrigger message.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_SendGoal_Response__copy(
  const jai_rosbridge__action__HDRTrigger_SendGoal_Response * input,
  jai_rosbridge__action__HDRTrigger_SendGoal_Response * output);

/// Initialize array of action/HDRTrigger messages.
/**
 * It allocates the memory for the number of elements and calls
 * jai_rosbridge__action__HDRTrigger_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__init(jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__fini(jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * array);

/// Create array of action/HDRTrigger messages.
/**
 * It allocates the memory for the array and calls
 * jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence *
jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__destroy(jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * array);

/// Check for action/HDRTrigger message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/HDRTrigger messages.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * input,
  jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence * output);

/// Initialize action/HDRTrigger message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * jai_rosbridge__action__HDRTrigger_GetResult_Request
 * )) before or use
 * jai_rosbridge__action__HDRTrigger_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_GetResult_Request__init(jai_rosbridge__action__HDRTrigger_GetResult_Request * msg);

/// Finalize action/HDRTrigger message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_GetResult_Request__fini(jai_rosbridge__action__HDRTrigger_GetResult_Request * msg);

/// Create action/HDRTrigger message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * jai_rosbridge__action__HDRTrigger_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_GetResult_Request *
jai_rosbridge__action__HDRTrigger_GetResult_Request__create();

/// Destroy action/HDRTrigger message.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_GetResult_Request__destroy(jai_rosbridge__action__HDRTrigger_GetResult_Request * msg);

/// Check for action/HDRTrigger message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_GetResult_Request__are_equal(const jai_rosbridge__action__HDRTrigger_GetResult_Request * lhs, const jai_rosbridge__action__HDRTrigger_GetResult_Request * rhs);

/// Copy a action/HDRTrigger message.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_GetResult_Request__copy(
  const jai_rosbridge__action__HDRTrigger_GetResult_Request * input,
  jai_rosbridge__action__HDRTrigger_GetResult_Request * output);

/// Initialize array of action/HDRTrigger messages.
/**
 * It allocates the memory for the number of elements and calls
 * jai_rosbridge__action__HDRTrigger_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__init(jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__fini(jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * array);

/// Create array of action/HDRTrigger messages.
/**
 * It allocates the memory for the array and calls
 * jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence *
jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__destroy(jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * array);

/// Check for action/HDRTrigger message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * rhs);

/// Copy an array of action/HDRTrigger messages.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * input,
  jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence * output);

/// Initialize action/HDRTrigger message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * jai_rosbridge__action__HDRTrigger_GetResult_Response
 * )) before or use
 * jai_rosbridge__action__HDRTrigger_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_GetResult_Response__init(jai_rosbridge__action__HDRTrigger_GetResult_Response * msg);

/// Finalize action/HDRTrigger message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_GetResult_Response__fini(jai_rosbridge__action__HDRTrigger_GetResult_Response * msg);

/// Create action/HDRTrigger message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * jai_rosbridge__action__HDRTrigger_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_GetResult_Response *
jai_rosbridge__action__HDRTrigger_GetResult_Response__create();

/// Destroy action/HDRTrigger message.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_GetResult_Response__destroy(jai_rosbridge__action__HDRTrigger_GetResult_Response * msg);

/// Check for action/HDRTrigger message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_GetResult_Response__are_equal(const jai_rosbridge__action__HDRTrigger_GetResult_Response * lhs, const jai_rosbridge__action__HDRTrigger_GetResult_Response * rhs);

/// Copy a action/HDRTrigger message.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_GetResult_Response__copy(
  const jai_rosbridge__action__HDRTrigger_GetResult_Response * input,
  jai_rosbridge__action__HDRTrigger_GetResult_Response * output);

/// Initialize array of action/HDRTrigger messages.
/**
 * It allocates the memory for the number of elements and calls
 * jai_rosbridge__action__HDRTrigger_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__init(jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__fini(jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * array);

/// Create array of action/HDRTrigger messages.
/**
 * It allocates the memory for the array and calls
 * jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence *
jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__destroy(jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * array);

/// Check for action/HDRTrigger message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * rhs);

/// Copy an array of action/HDRTrigger messages.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * input,
  jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence * output);

/// Initialize action/HDRTrigger message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * jai_rosbridge__action__HDRTrigger_FeedbackMessage
 * )) before or use
 * jai_rosbridge__action__HDRTrigger_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_FeedbackMessage__init(jai_rosbridge__action__HDRTrigger_FeedbackMessage * msg);

/// Finalize action/HDRTrigger message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_FeedbackMessage__fini(jai_rosbridge__action__HDRTrigger_FeedbackMessage * msg);

/// Create action/HDRTrigger message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * jai_rosbridge__action__HDRTrigger_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_FeedbackMessage *
jai_rosbridge__action__HDRTrigger_FeedbackMessage__create();

/// Destroy action/HDRTrigger message.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_FeedbackMessage__destroy(jai_rosbridge__action__HDRTrigger_FeedbackMessage * msg);

/// Check for action/HDRTrigger message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_FeedbackMessage__are_equal(const jai_rosbridge__action__HDRTrigger_FeedbackMessage * lhs, const jai_rosbridge__action__HDRTrigger_FeedbackMessage * rhs);

/// Copy a action/HDRTrigger message.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_FeedbackMessage__copy(
  const jai_rosbridge__action__HDRTrigger_FeedbackMessage * input,
  jai_rosbridge__action__HDRTrigger_FeedbackMessage * output);

/// Initialize array of action/HDRTrigger messages.
/**
 * It allocates the memory for the number of elements and calls
 * jai_rosbridge__action__HDRTrigger_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__init(jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__fini(jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * array);

/// Create array of action/HDRTrigger messages.
/**
 * It allocates the memory for the array and calls
 * jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence *
jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/HDRTrigger messages.
/**
 * It calls
 * jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
void
jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__destroy(jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * array);

/// Check for action/HDRTrigger message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__are_equal(const jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * lhs, const jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/HDRTrigger messages.
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
ROSIDL_GENERATOR_C_PUBLIC_jai_rosbridge
bool
jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence__copy(
  const jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * input,
  jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__FUNCTIONS_H_
