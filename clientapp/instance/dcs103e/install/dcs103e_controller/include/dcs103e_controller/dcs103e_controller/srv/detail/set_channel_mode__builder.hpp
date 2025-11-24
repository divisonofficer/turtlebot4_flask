// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dcs103e_controller:srv/SetChannelMode.idl
// generated code does not contain a copyright notice

#ifndef DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__BUILDER_HPP_
#define DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dcs103e_controller/srv/detail/set_channel_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dcs103e_controller
{

namespace srv
{

namespace builder
{

class Init_SetChannelMode_Request_pulse_delay
{
public:
  explicit Init_SetChannelMode_Request_pulse_delay(::dcs103e_controller::srv::SetChannelMode_Request & msg)
  : msg_(msg)
  {}
  ::dcs103e_controller::srv::SetChannelMode_Request pulse_delay(::dcs103e_controller::srv::SetChannelMode_Request::_pulse_delay_type arg)
  {
    msg_.pulse_delay = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dcs103e_controller::srv::SetChannelMode_Request msg_;
};

class Init_SetChannelMode_Request_pulse_width
{
public:
  explicit Init_SetChannelMode_Request_pulse_width(::dcs103e_controller::srv::SetChannelMode_Request & msg)
  : msg_(msg)
  {}
  Init_SetChannelMode_Request_pulse_delay pulse_width(::dcs103e_controller::srv::SetChannelMode_Request::_pulse_width_type arg)
  {
    msg_.pulse_width = std::move(arg);
    return Init_SetChannelMode_Request_pulse_delay(msg_);
  }

private:
  ::dcs103e_controller::srv::SetChannelMode_Request msg_;
};

class Init_SetChannelMode_Request_current
{
public:
  explicit Init_SetChannelMode_Request_current(::dcs103e_controller::srv::SetChannelMode_Request & msg)
  : msg_(msg)
  {}
  Init_SetChannelMode_Request_pulse_width current(::dcs103e_controller::srv::SetChannelMode_Request::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_SetChannelMode_Request_pulse_width(msg_);
  }

private:
  ::dcs103e_controller::srv::SetChannelMode_Request msg_;
};

class Init_SetChannelMode_Request_mode
{
public:
  explicit Init_SetChannelMode_Request_mode(::dcs103e_controller::srv::SetChannelMode_Request & msg)
  : msg_(msg)
  {}
  Init_SetChannelMode_Request_current mode(::dcs103e_controller::srv::SetChannelMode_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_SetChannelMode_Request_current(msg_);
  }

private:
  ::dcs103e_controller::srv::SetChannelMode_Request msg_;
};

class Init_SetChannelMode_Request_channel
{
public:
  Init_SetChannelMode_Request_channel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetChannelMode_Request_mode channel(::dcs103e_controller::srv::SetChannelMode_Request::_channel_type arg)
  {
    msg_.channel = std::move(arg);
    return Init_SetChannelMode_Request_mode(msg_);
  }

private:
  ::dcs103e_controller::srv::SetChannelMode_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dcs103e_controller::srv::SetChannelMode_Request>()
{
  return dcs103e_controller::srv::builder::Init_SetChannelMode_Request_channel();
}

}  // namespace dcs103e_controller


namespace dcs103e_controller
{

namespace srv
{

namespace builder
{

class Init_SetChannelMode_Response_message
{
public:
  explicit Init_SetChannelMode_Response_message(::dcs103e_controller::srv::SetChannelMode_Response & msg)
  : msg_(msg)
  {}
  ::dcs103e_controller::srv::SetChannelMode_Response message(::dcs103e_controller::srv::SetChannelMode_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dcs103e_controller::srv::SetChannelMode_Response msg_;
};

class Init_SetChannelMode_Response_success
{
public:
  Init_SetChannelMode_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetChannelMode_Response_message success(::dcs103e_controller::srv::SetChannelMode_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetChannelMode_Response_message(msg_);
  }

private:
  ::dcs103e_controller::srv::SetChannelMode_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dcs103e_controller::srv::SetChannelMode_Response>()
{
  return dcs103e_controller::srv::builder::Init_SetChannelMode_Response_success();
}

}  // namespace dcs103e_controller

#endif  // DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__BUILDER_HPP_
