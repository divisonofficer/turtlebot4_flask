// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from piper_msgs:srv/MoveArm.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__MOVE_ARM__STRUCT_H_
#define PIPER_MSGS__SRV__DETAIL__MOVE_ARM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/MoveArm in the package piper_msgs.
typedef struct piper_msgs__srv__MoveArm_Request
{
  int32_t move_idx;
} piper_msgs__srv__MoveArm_Request;

// Struct for a sequence of piper_msgs__srv__MoveArm_Request.
typedef struct piper_msgs__srv__MoveArm_Request__Sequence
{
  piper_msgs__srv__MoveArm_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} piper_msgs__srv__MoveArm_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MoveArm in the package piper_msgs.
typedef struct piper_msgs__srv__MoveArm_Response
{
  bool success;
} piper_msgs__srv__MoveArm_Response;

// Struct for a sequence of piper_msgs__srv__MoveArm_Response.
typedef struct piper_msgs__srv__MoveArm_Response__Sequence
{
  piper_msgs__srv__MoveArm_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} piper_msgs__srv__MoveArm_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PIPER_MSGS__SRV__DETAIL__MOVE_ARM__STRUCT_H_
