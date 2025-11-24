// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dcs103e_controller:srv/ControlChannel.idl
// generated code does not contain a copyright notice

#ifndef DCS103E_CONTROLLER__SRV__DETAIL__CONTROL_CHANNEL__STRUCT_H_
#define DCS103E_CONTROLLER__SRV__DETAIL__CONTROL_CHANNEL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ControlChannel in the package dcs103e_controller.
typedef struct dcs103e_controller__srv__ControlChannel_Request
{
  /// Channel number (0, 1, 2)
  int32_t channel;
  /// true to turn on, false to turn off
  bool enable;
  /// Current value (optional, used when enabling)
  double current;
} dcs103e_controller__srv__ControlChannel_Request;

// Struct for a sequence of dcs103e_controller__srv__ControlChannel_Request.
typedef struct dcs103e_controller__srv__ControlChannel_Request__Sequence
{
  dcs103e_controller__srv__ControlChannel_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dcs103e_controller__srv__ControlChannel_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ControlChannel in the package dcs103e_controller.
typedef struct dcs103e_controller__srv__ControlChannel_Response
{
  bool success;
  rosidl_runtime_c__String message;
} dcs103e_controller__srv__ControlChannel_Response;

// Struct for a sequence of dcs103e_controller__srv__ControlChannel_Response.
typedef struct dcs103e_controller__srv__ControlChannel_Response__Sequence
{
  dcs103e_controller__srv__ControlChannel_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dcs103e_controller__srv__ControlChannel_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DCS103E_CONTROLLER__SRV__DETAIL__CONTROL_CHANNEL__STRUCT_H_
