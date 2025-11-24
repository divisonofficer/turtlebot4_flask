// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dcs103e_controller:srv/SetChannelMode.idl
// generated code does not contain a copyright notice

#ifndef DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__TRAITS_HPP_
#define DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dcs103e_controller/srv/detail/set_channel_mode__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dcs103e_controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetChannelMode_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: channel
  {
    out << "channel: ";
    rosidl_generator_traits::value_to_yaml(msg.channel, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: current
  {
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << ", ";
  }

  // member: pulse_width
  {
    out << "pulse_width: ";
    rosidl_generator_traits::value_to_yaml(msg.pulse_width, out);
    out << ", ";
  }

  // member: pulse_delay
  {
    out << "pulse_delay: ";
    rosidl_generator_traits::value_to_yaml(msg.pulse_delay, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetChannelMode_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: channel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel: ";
    rosidl_generator_traits::value_to_yaml(msg.channel, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << "\n";
  }

  // member: pulse_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pulse_width: ";
    rosidl_generator_traits::value_to_yaml(msg.pulse_width, out);
    out << "\n";
  }

  // member: pulse_delay
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pulse_delay: ";
    rosidl_generator_traits::value_to_yaml(msg.pulse_delay, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetChannelMode_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace dcs103e_controller

namespace rosidl_generator_traits
{

[[deprecated("use dcs103e_controller::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dcs103e_controller::srv::SetChannelMode_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dcs103e_controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dcs103e_controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const dcs103e_controller::srv::SetChannelMode_Request & msg)
{
  return dcs103e_controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dcs103e_controller::srv::SetChannelMode_Request>()
{
  return "dcs103e_controller::srv::SetChannelMode_Request";
}

template<>
inline const char * name<dcs103e_controller::srv::SetChannelMode_Request>()
{
  return "dcs103e_controller/srv/SetChannelMode_Request";
}

template<>
struct has_fixed_size<dcs103e_controller::srv::SetChannelMode_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dcs103e_controller::srv::SetChannelMode_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dcs103e_controller::srv::SetChannelMode_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace dcs103e_controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetChannelMode_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetChannelMode_Response & msg,
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

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetChannelMode_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace dcs103e_controller

namespace rosidl_generator_traits
{

[[deprecated("use dcs103e_controller::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dcs103e_controller::srv::SetChannelMode_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dcs103e_controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dcs103e_controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const dcs103e_controller::srv::SetChannelMode_Response & msg)
{
  return dcs103e_controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dcs103e_controller::srv::SetChannelMode_Response>()
{
  return "dcs103e_controller::srv::SetChannelMode_Response";
}

template<>
inline const char * name<dcs103e_controller::srv::SetChannelMode_Response>()
{
  return "dcs103e_controller/srv/SetChannelMode_Response";
}

template<>
struct has_fixed_size<dcs103e_controller::srv::SetChannelMode_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dcs103e_controller::srv::SetChannelMode_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dcs103e_controller::srv::SetChannelMode_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dcs103e_controller::srv::SetChannelMode>()
{
  return "dcs103e_controller::srv::SetChannelMode";
}

template<>
inline const char * name<dcs103e_controller::srv::SetChannelMode>()
{
  return "dcs103e_controller/srv/SetChannelMode";
}

template<>
struct has_fixed_size<dcs103e_controller::srv::SetChannelMode>
  : std::integral_constant<
    bool,
    has_fixed_size<dcs103e_controller::srv::SetChannelMode_Request>::value &&
    has_fixed_size<dcs103e_controller::srv::SetChannelMode_Response>::value
  >
{
};

template<>
struct has_bounded_size<dcs103e_controller::srv::SetChannelMode>
  : std::integral_constant<
    bool,
    has_bounded_size<dcs103e_controller::srv::SetChannelMode_Request>::value &&
    has_bounded_size<dcs103e_controller::srv::SetChannelMode_Response>::value
  >
{
};

template<>
struct is_service<dcs103e_controller::srv::SetChannelMode>
  : std::true_type
{
};

template<>
struct is_service_request<dcs103e_controller::srv::SetChannelMode_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dcs103e_controller::srv::SetChannelMode_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__TRAITS_HPP_
