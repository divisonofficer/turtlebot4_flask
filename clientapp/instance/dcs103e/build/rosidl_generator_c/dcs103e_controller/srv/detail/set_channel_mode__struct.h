// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dcs103e_controller:srv/SetChannelMode.idl
// generated code does not contain a copyright notice

#ifndef DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__STRUCT_H_
#define DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'mode'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetChannelMode in the package dcs103e_controller.
typedef struct dcs103e_controller__srv__SetChannelMode_Request
{
  /// Channel number (0, 1, 2)
  int32_t channel;
  /// "continuous" or "pulsed"
  rosidl_runtime_c__String mode;
  /// Current value
  double current;
  /// Pulse width (for pulsed mode only)
  double pulse_width;
  /// Pulse delay (for pulsed mode only)
  double pulse_delay;
} dcs103e_controller__srv__SetChannelMode_Request;

// Struct for a sequence of dcs103e_controller__srv__SetChannelMode_Request.
typedef struct dcs103e_controller__srv__SetChannelMode_Request__Sequence
{
  dcs103e_controller__srv__SetChannelMode_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dcs103e_controller__srv__SetChannelMode_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetChannelMode in the package dcs103e_controller.
typedef struct dcs103e_controller__srv__SetChannelMode_Response
{
  bool success;
  rosidl_runtime_c__String message;
} dcs103e_controller__srv__SetChannelMode_Response;

// Struct for a sequence of dcs103e_controller__srv__SetChannelMode_Response.
typedef struct dcs103e_controller__srv__SetChannelMode_Response__Sequence
{
  dcs103e_controller__srv__SetChannelMode_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dcs103e_controller__srv__SetChannelMode_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__STRUCT_H_
