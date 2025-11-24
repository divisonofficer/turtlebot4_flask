// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dcs103e_controller:srv/SetChannelMode.idl
// generated code does not contain a copyright notice

#ifndef DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__FUNCTIONS_H_
#define DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dcs103e_controller/msg/rosidl_generator_c__visibility_control.h"

#include "dcs103e_controller/srv/detail/set_channel_mode__struct.h"

/// Initialize srv/SetChannelMode message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dcs103e_controller__srv__SetChannelMode_Request
 * )) before or use
 * dcs103e_controller__srv__SetChannelMode_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
bool
dcs103e_controller__srv__SetChannelMode_Request__init(dcs103e_controller__srv__SetChannelMode_Request * msg);

/// Finalize srv/SetChannelMode message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
void
dcs103e_controller__srv__SetChannelMode_Request__fini(dcs103e_controller__srv__SetChannelMode_Request * msg);

/// Create srv/SetChannelMode message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dcs103e_controller__srv__SetChannelMode_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
dcs103e_controller__srv__SetChannelMode_Request *
dcs103e_controller__srv__SetChannelMode_Request__create();

/// Destroy srv/SetChannelMode message.
/**
 * It calls
 * dcs103e_controller__srv__SetChannelMode_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
void
dcs103e_controller__srv__SetChannelMode_Request__destroy(dcs103e_controller__srv__SetChannelMode_Request * msg);

/// Check for srv/SetChannelMode message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
bool
dcs103e_controller__srv__SetChannelMode_Request__are_equal(const dcs103e_controller__srv__SetChannelMode_Request * lhs, const dcs103e_controller__srv__SetChannelMode_Request * rhs);

/// Copy a srv/SetChannelMode message.
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
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
bool
dcs103e_controller__srv__SetChannelMode_Request__copy(
  const dcs103e_controller__srv__SetChannelMode_Request * input,
  dcs103e_controller__srv__SetChannelMode_Request * output);

/// Initialize array of srv/SetChannelMode messages.
/**
 * It allocates the memory for the number of elements and calls
 * dcs103e_controller__srv__SetChannelMode_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
bool
dcs103e_controller__srv__SetChannelMode_Request__Sequence__init(dcs103e_controller__srv__SetChannelMode_Request__Sequence * array, size_t size);

/// Finalize array of srv/SetChannelMode messages.
/**
 * It calls
 * dcs103e_controller__srv__SetChannelMode_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
void
dcs103e_controller__srv__SetChannelMode_Request__Sequence__fini(dcs103e_controller__srv__SetChannelMode_Request__Sequence * array);

/// Create array of srv/SetChannelMode messages.
/**
 * It allocates the memory for the array and calls
 * dcs103e_controller__srv__SetChannelMode_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
dcs103e_controller__srv__SetChannelMode_Request__Sequence *
dcs103e_controller__srv__SetChannelMode_Request__Sequence__create(size_t size);

/// Destroy array of srv/SetChannelMode messages.
/**
 * It calls
 * dcs103e_controller__srv__SetChannelMode_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
void
dcs103e_controller__srv__SetChannelMode_Request__Sequence__destroy(dcs103e_controller__srv__SetChannelMode_Request__Sequence * array);

/// Check for srv/SetChannelMode message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
bool
dcs103e_controller__srv__SetChannelMode_Request__Sequence__are_equal(const dcs103e_controller__srv__SetChannelMode_Request__Sequence * lhs, const dcs103e_controller__srv__SetChannelMode_Request__Sequence * rhs);

/// Copy an array of srv/SetChannelMode messages.
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
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
bool
dcs103e_controller__srv__SetChannelMode_Request__Sequence__copy(
  const dcs103e_controller__srv__SetChannelMode_Request__Sequence * input,
  dcs103e_controller__srv__SetChannelMode_Request__Sequence * output);

/// Initialize srv/SetChannelMode message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dcs103e_controller__srv__SetChannelMode_Response
 * )) before or use
 * dcs103e_controller__srv__SetChannelMode_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
bool
dcs103e_controller__srv__SetChannelMode_Response__init(dcs103e_controller__srv__SetChannelMode_Response * msg);

/// Finalize srv/SetChannelMode message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
void
dcs103e_controller__srv__SetChannelMode_Response__fini(dcs103e_controller__srv__SetChannelMode_Response * msg);

/// Create srv/SetChannelMode message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dcs103e_controller__srv__SetChannelMode_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
dcs103e_controller__srv__SetChannelMode_Response *
dcs103e_controller__srv__SetChannelMode_Response__create();

/// Destroy srv/SetChannelMode message.
/**
 * It calls
 * dcs103e_controller__srv__SetChannelMode_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
void
dcs103e_controller__srv__SetChannelMode_Response__destroy(dcs103e_controller__srv__SetChannelMode_Response * msg);

/// Check for srv/SetChannelMode message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
bool
dcs103e_controller__srv__SetChannelMode_Response__are_equal(const dcs103e_controller__srv__SetChannelMode_Response * lhs, const dcs103e_controller__srv__SetChannelMode_Response * rhs);

/// Copy a srv/SetChannelMode message.
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
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
bool
dcs103e_controller__srv__SetChannelMode_Response__copy(
  const dcs103e_controller__srv__SetChannelMode_Response * input,
  dcs103e_controller__srv__SetChannelMode_Response * output);

/// Initialize array of srv/SetChannelMode messages.
/**
 * It allocates the memory for the number of elements and calls
 * dcs103e_controller__srv__SetChannelMode_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
bool
dcs103e_controller__srv__SetChannelMode_Response__Sequence__init(dcs103e_controller__srv__SetChannelMode_Response__Sequence * array, size_t size);

/// Finalize array of srv/SetChannelMode messages.
/**
 * It calls
 * dcs103e_controller__srv__SetChannelMode_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
void
dcs103e_controller__srv__SetChannelMode_Response__Sequence__fini(dcs103e_controller__srv__SetChannelMode_Response__Sequence * array);

/// Create array of srv/SetChannelMode messages.
/**
 * It allocates the memory for the array and calls
 * dcs103e_controller__srv__SetChannelMode_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
dcs103e_controller__srv__SetChannelMode_Response__Sequence *
dcs103e_controller__srv__SetChannelMode_Response__Sequence__create(size_t size);

/// Destroy array of srv/SetChannelMode messages.
/**
 * It calls
 * dcs103e_controller__srv__SetChannelMode_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
void
dcs103e_controller__srv__SetChannelMode_Response__Sequence__destroy(dcs103e_controller__srv__SetChannelMode_Response__Sequence * array);

/// Check for srv/SetChannelMode message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
bool
dcs103e_controller__srv__SetChannelMode_Response__Sequence__are_equal(const dcs103e_controller__srv__SetChannelMode_Response__Sequence * lhs, const dcs103e_controller__srv__SetChannelMode_Response__Sequence * rhs);

/// Copy an array of srv/SetChannelMode messages.
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
ROSIDL_GENERATOR_C_PUBLIC_dcs103e_controller
bool
dcs103e_controller__srv__SetChannelMode_Response__Sequence__copy(
  const dcs103e_controller__srv__SetChannelMode_Response__Sequence * input,
  dcs103e_controller__srv__SetChannelMode_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__FUNCTIONS_H_
