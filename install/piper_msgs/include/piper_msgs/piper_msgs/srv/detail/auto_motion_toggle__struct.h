// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from piper_msgs:srv/AutoMotionToggle.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__AUTO_MOTION_TOGGLE__STRUCT_H_
#define PIPER_MSGS__SRV__DETAIL__AUTO_MOTION_TOGGLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/AutoMotionToggle in the package piper_msgs.
typedef struct piper_msgs__srv__AutoMotionToggle_Request
{
  bool enable;
} piper_msgs__srv__AutoMotionToggle_Request;

// Struct for a sequence of piper_msgs__srv__AutoMotionToggle_Request.
typedef struct piper_msgs__srv__AutoMotionToggle_Request__Sequence
{
  piper_msgs__srv__AutoMotionToggle_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} piper_msgs__srv__AutoMotionToggle_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/AutoMotionToggle in the package piper_msgs.
typedef struct piper_msgs__srv__AutoMotionToggle_Response
{
  bool success;
} piper_msgs__srv__AutoMotionToggle_Response;

// Struct for a sequence of piper_msgs__srv__AutoMotionToggle_Response.
typedef struct piper_msgs__srv__AutoMotionToggle_Response__Sequence
{
  piper_msgs__srv__AutoMotionToggle_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} piper_msgs__srv__AutoMotionToggle_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PIPER_MSGS__SRV__DETAIL__AUTO_MOTION_TOGGLE__STRUCT_H_
