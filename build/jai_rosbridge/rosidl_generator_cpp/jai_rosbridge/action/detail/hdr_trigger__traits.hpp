// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jai_rosbridge:action/HDRTrigger.idl
// generated code does not contain a copyright notice

#ifndef JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__TRAITS_HPP_
#define JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jai_rosbridge/action/detail/hdr_trigger__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace jai_rosbridge
{

namespace action
{

inline void to_flow_style_yaml(
  const HDRTrigger_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: space_id
  {
    out << "space_id: ";
    rosidl_generator_traits::value_to_yaml(msg.space_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HDRTrigger_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: space_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "space_id: ";
    rosidl_generator_traits::value_to_yaml(msg.space_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HDRTrigger_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace jai_rosbridge

namespace rosidl_generator_traits
{

[[deprecated("use jai_rosbridge::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const jai_rosbridge::action::HDRTrigger_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  jai_rosbridge::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jai_rosbridge::action::to_yaml() instead")]]
inline std::string to_yaml(const jai_rosbridge::action::HDRTrigger_Goal & msg)
{
  return jai_rosbridge::action::to_yaml(msg);
}

template<>
inline const char * data_type<jai_rosbridge::action::HDRTrigger_Goal>()
{
  return "jai_rosbridge::action::HDRTrigger_Goal";
}

template<>
inline const char * name<jai_rosbridge::action::HDRTrigger_Goal>()
{
  return "jai_rosbridge/action/HDRTrigger_Goal";
}

template<>
struct has_fixed_size<jai_rosbridge::action::HDRTrigger_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<jai_rosbridge::action::HDRTrigger_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<jai_rosbridge::action::HDRTrigger_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace jai_rosbridge
{

namespace action
{

inline void to_flow_style_yaml(
  const HDRTrigger_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: result_message
  {
    out << "result_message: ";
    rosidl_generator_traits::value_to_yaml(msg.result_message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HDRTrigger_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: result_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result_message: ";
    rosidl_generator_traits::value_to_yaml(msg.result_message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HDRTrigger_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace jai_rosbridge

namespace rosidl_generator_traits
{

[[deprecated("use jai_rosbridge::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const jai_rosbridge::action::HDRTrigger_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  jai_rosbridge::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jai_rosbridge::action::to_yaml() instead")]]
inline std::string to_yaml(const jai_rosbridge::action::HDRTrigger_Result & msg)
{
  return jai_rosbridge::action::to_yaml(msg);
}

template<>
inline const char * data_type<jai_rosbridge::action::HDRTrigger_Result>()
{
  return "jai_rosbridge::action::HDRTrigger_Result";
}

template<>
inline const char * name<jai_rosbridge::action::HDRTrigger_Result>()
{
  return "jai_rosbridge/action/HDRTrigger_Result";
}

template<>
struct has_fixed_size<jai_rosbridge::action::HDRTrigger_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<jai_rosbridge::action::HDRTrigger_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<jai_rosbridge::action::HDRTrigger_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace jai_rosbridge
{

namespace action
{

inline void to_flow_style_yaml(
  const HDRTrigger_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: feedback_message
  {
    out << "feedback_message: ";
    rosidl_generator_traits::value_to_yaml(msg.feedback_message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HDRTrigger_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: feedback_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback_message: ";
    rosidl_generator_traits::value_to_yaml(msg.feedback_message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HDRTrigger_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace jai_rosbridge

namespace rosidl_generator_traits
{

[[deprecated("use jai_rosbridge::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const jai_rosbridge::action::HDRTrigger_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  jai_rosbridge::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jai_rosbridge::action::to_yaml() instead")]]
inline std::string to_yaml(const jai_rosbridge::action::HDRTrigger_Feedback & msg)
{
  return jai_rosbridge::action::to_yaml(msg);
}

template<>
inline const char * data_type<jai_rosbridge::action::HDRTrigger_Feedback>()
{
  return "jai_rosbridge::action::HDRTrigger_Feedback";
}

template<>
inline const char * name<jai_rosbridge::action::HDRTrigger_Feedback>()
{
  return "jai_rosbridge/action/HDRTrigger_Feedback";
}

template<>
struct has_fixed_size<jai_rosbridge::action::HDRTrigger_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<jai_rosbridge::action::HDRTrigger_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<jai_rosbridge::action::HDRTrigger_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "jai_rosbridge/action/detail/hdr_trigger__traits.hpp"

namespace jai_rosbridge
{

namespace action
{

inline void to_flow_style_yaml(
  const HDRTrigger_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HDRTrigger_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HDRTrigger_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace jai_rosbridge

namespace rosidl_generator_traits
{

[[deprecated("use jai_rosbridge::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const jai_rosbridge::action::HDRTrigger_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  jai_rosbridge::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jai_rosbridge::action::to_yaml() instead")]]
inline std::string to_yaml(const jai_rosbridge::action::HDRTrigger_SendGoal_Request & msg)
{
  return jai_rosbridge::action::to_yaml(msg);
}

template<>
inline const char * data_type<jai_rosbridge::action::HDRTrigger_SendGoal_Request>()
{
  return "jai_rosbridge::action::HDRTrigger_SendGoal_Request";
}

template<>
inline const char * name<jai_rosbridge::action::HDRTrigger_SendGoal_Request>()
{
  return "jai_rosbridge/action/HDRTrigger_SendGoal_Request";
}

template<>
struct has_fixed_size<jai_rosbridge::action::HDRTrigger_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<jai_rosbridge::action::HDRTrigger_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<jai_rosbridge::action::HDRTrigger_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<jai_rosbridge::action::HDRTrigger_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<jai_rosbridge::action::HDRTrigger_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace jai_rosbridge
{

namespace action
{

inline void to_flow_style_yaml(
  const HDRTrigger_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HDRTrigger_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HDRTrigger_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace jai_rosbridge

namespace rosidl_generator_traits
{

[[deprecated("use jai_rosbridge::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const jai_rosbridge::action::HDRTrigger_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  jai_rosbridge::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jai_rosbridge::action::to_yaml() instead")]]
inline std::string to_yaml(const jai_rosbridge::action::HDRTrigger_SendGoal_Response & msg)
{
  return jai_rosbridge::action::to_yaml(msg);
}

template<>
inline const char * data_type<jai_rosbridge::action::HDRTrigger_SendGoal_Response>()
{
  return "jai_rosbridge::action::HDRTrigger_SendGoal_Response";
}

template<>
inline const char * name<jai_rosbridge::action::HDRTrigger_SendGoal_Response>()
{
  return "jai_rosbridge/action/HDRTrigger_SendGoal_Response";
}

template<>
struct has_fixed_size<jai_rosbridge::action::HDRTrigger_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<jai_rosbridge::action::HDRTrigger_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<jai_rosbridge::action::HDRTrigger_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<jai_rosbridge::action::HDRTrigger_SendGoal>()
{
  return "jai_rosbridge::action::HDRTrigger_SendGoal";
}

template<>
inline const char * name<jai_rosbridge::action::HDRTrigger_SendGoal>()
{
  return "jai_rosbridge/action/HDRTrigger_SendGoal";
}

template<>
struct has_fixed_size<jai_rosbridge::action::HDRTrigger_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<jai_rosbridge::action::HDRTrigger_SendGoal_Request>::value &&
    has_fixed_size<jai_rosbridge::action::HDRTrigger_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<jai_rosbridge::action::HDRTrigger_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<jai_rosbridge::action::HDRTrigger_SendGoal_Request>::value &&
    has_bounded_size<jai_rosbridge::action::HDRTrigger_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<jai_rosbridge::action::HDRTrigger_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<jai_rosbridge::action::HDRTrigger_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<jai_rosbridge::action::HDRTrigger_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace jai_rosbridge
{

namespace action
{

inline void to_flow_style_yaml(
  const HDRTrigger_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HDRTrigger_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HDRTrigger_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace jai_rosbridge

namespace rosidl_generator_traits
{

[[deprecated("use jai_rosbridge::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const jai_rosbridge::action::HDRTrigger_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  jai_rosbridge::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jai_rosbridge::action::to_yaml() instead")]]
inline std::string to_yaml(const jai_rosbridge::action::HDRTrigger_GetResult_Request & msg)
{
  return jai_rosbridge::action::to_yaml(msg);
}

template<>
inline const char * data_type<jai_rosbridge::action::HDRTrigger_GetResult_Request>()
{
  return "jai_rosbridge::action::HDRTrigger_GetResult_Request";
}

template<>
inline const char * name<jai_rosbridge::action::HDRTrigger_GetResult_Request>()
{
  return "jai_rosbridge/action/HDRTrigger_GetResult_Request";
}

template<>
struct has_fixed_size<jai_rosbridge::action::HDRTrigger_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<jai_rosbridge::action::HDRTrigger_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<jai_rosbridge::action::HDRTrigger_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "jai_rosbridge/action/detail/hdr_trigger__traits.hpp"

namespace jai_rosbridge
{

namespace action
{

inline void to_flow_style_yaml(
  const HDRTrigger_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HDRTrigger_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HDRTrigger_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace jai_rosbridge

namespace rosidl_generator_traits
{

[[deprecated("use jai_rosbridge::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const jai_rosbridge::action::HDRTrigger_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  jai_rosbridge::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jai_rosbridge::action::to_yaml() instead")]]
inline std::string to_yaml(const jai_rosbridge::action::HDRTrigger_GetResult_Response & msg)
{
  return jai_rosbridge::action::to_yaml(msg);
}

template<>
inline const char * data_type<jai_rosbridge::action::HDRTrigger_GetResult_Response>()
{
  return "jai_rosbridge::action::HDRTrigger_GetResult_Response";
}

template<>
inline const char * name<jai_rosbridge::action::HDRTrigger_GetResult_Response>()
{
  return "jai_rosbridge/action/HDRTrigger_GetResult_Response";
}

template<>
struct has_fixed_size<jai_rosbridge::action::HDRTrigger_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<jai_rosbridge::action::HDRTrigger_Result>::value> {};

template<>
struct has_bounded_size<jai_rosbridge::action::HDRTrigger_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<jai_rosbridge::action::HDRTrigger_Result>::value> {};

template<>
struct is_message<jai_rosbridge::action::HDRTrigger_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<jai_rosbridge::action::HDRTrigger_GetResult>()
{
  return "jai_rosbridge::action::HDRTrigger_GetResult";
}

template<>
inline const char * name<jai_rosbridge::action::HDRTrigger_GetResult>()
{
  return "jai_rosbridge/action/HDRTrigger_GetResult";
}

template<>
struct has_fixed_size<jai_rosbridge::action::HDRTrigger_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<jai_rosbridge::action::HDRTrigger_GetResult_Request>::value &&
    has_fixed_size<jai_rosbridge::action::HDRTrigger_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<jai_rosbridge::action::HDRTrigger_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<jai_rosbridge::action::HDRTrigger_GetResult_Request>::value &&
    has_bounded_size<jai_rosbridge::action::HDRTrigger_GetResult_Response>::value
  >
{
};

template<>
struct is_service<jai_rosbridge::action::HDRTrigger_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<jai_rosbridge::action::HDRTrigger_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<jai_rosbridge::action::HDRTrigger_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "jai_rosbridge/action/detail/hdr_trigger__traits.hpp"

namespace jai_rosbridge
{

namespace action
{

inline void to_flow_style_yaml(
  const HDRTrigger_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HDRTrigger_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HDRTrigger_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace jai_rosbridge

namespace rosidl_generator_traits
{

[[deprecated("use jai_rosbridge::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const jai_rosbridge::action::HDRTrigger_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  jai_rosbridge::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jai_rosbridge::action::to_yaml() instead")]]
inline std::string to_yaml(const jai_rosbridge::action::HDRTrigger_FeedbackMessage & msg)
{
  return jai_rosbridge::action::to_yaml(msg);
}

template<>
inline const char * data_type<jai_rosbridge::action::HDRTrigger_FeedbackMessage>()
{
  return "jai_rosbridge::action::HDRTrigger_FeedbackMessage";
}

template<>
inline const char * name<jai_rosbridge::action::HDRTrigger_FeedbackMessage>()
{
  return "jai_rosbridge/action/HDRTrigger_FeedbackMessage";
}

template<>
struct has_fixed_size<jai_rosbridge::action::HDRTrigger_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<jai_rosbridge::action::HDRTrigger_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<jai_rosbridge::action::HDRTrigger_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<jai_rosbridge::action::HDRTrigger_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<jai_rosbridge::action::HDRTrigger_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<jai_rosbridge::action::HDRTrigger>
  : std::true_type
{
};

template<>
struct is_action_goal<jai_rosbridge::action::HDRTrigger_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<jai_rosbridge::action::HDRTrigger_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<jai_rosbridge::action::HDRTrigger_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__TRAITS_HPP_
