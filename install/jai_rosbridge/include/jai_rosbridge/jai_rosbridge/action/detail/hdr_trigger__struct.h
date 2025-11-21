// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jai_rosbridge:action/HDRTrigger.idl
// generated code does not contain a copyright notice

#ifndef JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__STRUCT_H_
#define JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'space_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/HDRTrigger in the package jai_rosbridge.
typedef struct jai_rosbridge__action__HDRTrigger_Goal
{
  /// 전송할 로그 메시지
  rosidl_runtime_c__String space_id;
} jai_rosbridge__action__HDRTrigger_Goal;

// Struct for a sequence of jai_rosbridge__action__HDRTrigger_Goal.
typedef struct jai_rosbridge__action__HDRTrigger_Goal__Sequence
{
  jai_rosbridge__action__HDRTrigger_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jai_rosbridge__action__HDRTrigger_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result_message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/HDRTrigger in the package jai_rosbridge.
typedef struct jai_rosbridge__action__HDRTrigger_Result
{
  /// Result definition
  /// 성공 여부
  bool success;
  rosidl_runtime_c__String result_message;
} jai_rosbridge__action__HDRTrigger_Result;

// Struct for a sequence of jai_rosbridge__action__HDRTrigger_Result.
typedef struct jai_rosbridge__action__HDRTrigger_Result__Sequence
{
  jai_rosbridge__action__HDRTrigger_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jai_rosbridge__action__HDRTrigger_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'feedback_message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/HDRTrigger in the package jai_rosbridge.
typedef struct jai_rosbridge__action__HDRTrigger_Feedback
{
  /// Feedback definition
  /// 진행 상황에 대한 피드백 메시지
  rosidl_runtime_c__String feedback_message;
} jai_rosbridge__action__HDRTrigger_Feedback;

// Struct for a sequence of jai_rosbridge__action__HDRTrigger_Feedback.
typedef struct jai_rosbridge__action__HDRTrigger_Feedback__Sequence
{
  jai_rosbridge__action__HDRTrigger_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jai_rosbridge__action__HDRTrigger_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "jai_rosbridge/action/detail/hdr_trigger__struct.h"

/// Struct defined in action/HDRTrigger in the package jai_rosbridge.
typedef struct jai_rosbridge__action__HDRTrigger_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  jai_rosbridge__action__HDRTrigger_Goal goal;
} jai_rosbridge__action__HDRTrigger_SendGoal_Request;

// Struct for a sequence of jai_rosbridge__action__HDRTrigger_SendGoal_Request.
typedef struct jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence
{
  jai_rosbridge__action__HDRTrigger_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jai_rosbridge__action__HDRTrigger_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/HDRTrigger in the package jai_rosbridge.
typedef struct jai_rosbridge__action__HDRTrigger_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} jai_rosbridge__action__HDRTrigger_SendGoal_Response;

// Struct for a sequence of jai_rosbridge__action__HDRTrigger_SendGoal_Response.
typedef struct jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence
{
  jai_rosbridge__action__HDRTrigger_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jai_rosbridge__action__HDRTrigger_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/HDRTrigger in the package jai_rosbridge.
typedef struct jai_rosbridge__action__HDRTrigger_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} jai_rosbridge__action__HDRTrigger_GetResult_Request;

// Struct for a sequence of jai_rosbridge__action__HDRTrigger_GetResult_Request.
typedef struct jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence
{
  jai_rosbridge__action__HDRTrigger_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jai_rosbridge__action__HDRTrigger_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "jai_rosbridge/action/detail/hdr_trigger__struct.h"

/// Struct defined in action/HDRTrigger in the package jai_rosbridge.
typedef struct jai_rosbridge__action__HDRTrigger_GetResult_Response
{
  int8_t status;
  jai_rosbridge__action__HDRTrigger_Result result;
} jai_rosbridge__action__HDRTrigger_GetResult_Response;

// Struct for a sequence of jai_rosbridge__action__HDRTrigger_GetResult_Response.
typedef struct jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence
{
  jai_rosbridge__action__HDRTrigger_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jai_rosbridge__action__HDRTrigger_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "jai_rosbridge/action/detail/hdr_trigger__struct.h"

/// Struct defined in action/HDRTrigger in the package jai_rosbridge.
typedef struct jai_rosbridge__action__HDRTrigger_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  jai_rosbridge__action__HDRTrigger_Feedback feedback;
} jai_rosbridge__action__HDRTrigger_FeedbackMessage;

// Struct for a sequence of jai_rosbridge__action__HDRTrigger_FeedbackMessage.
typedef struct jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence
{
  jai_rosbridge__action__HDRTrigger_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jai_rosbridge__action__HDRTrigger_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__STRUCT_H_
