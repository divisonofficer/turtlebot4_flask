// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dcs103e_controller:srv/ControlChannel.idl
// generated code does not contain a copyright notice

#ifndef DCS103E_CONTROLLER__SRV__DETAIL__CONTROL_CHANNEL__BUILDER_HPP_
#define DCS103E_CONTROLLER__SRV__DETAIL__CONTROL_CHANNEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dcs103e_controller/srv/detail/control_channel__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dcs103e_controller
{

namespace srv
{

namespace builder
{

class Init_ControlChannel_Request_current
{
public:
  explicit Init_ControlChannel_Request_current(::dcs103e_controller::srv::ControlChannel_Request & msg)
  : msg_(msg)
  {}
  ::dcs103e_controller::srv::ControlChannel_Request current(::dcs103e_controller::srv::ControlChannel_Request::_current_type arg)
  {
    msg_.current = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dcs103e_controller::srv::ControlChannel_Request msg_;
};

class Init_ControlChannel_Request_enable
{
public:
  explicit Init_ControlChannel_Request_enable(::dcs103e_controller::srv::ControlChannel_Request & msg)
  : msg_(msg)
  {}
  Init_ControlChannel_Request_current enable(::dcs103e_controller::srv::ControlChannel_Request::_enable_type arg)
  {
    msg_.enable = std::move(arg);
    return Init_ControlChannel_Request_current(msg_);
  }

private:
  ::dcs103e_controller::srv::ControlChannel_Request msg_;
};

class Init_ControlChannel_Request_channel
{
public:
  Init_ControlChannel_Request_channel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlChannel_Request_enable channel(::dcs103e_controller::srv::ControlChannel_Request::_channel_type arg)
  {
    msg_.channel = std::move(arg);
    return Init_ControlChannel_Request_enable(msg_);
  }

private:
  ::dcs103e_controller::srv::ControlChannel_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dcs103e_controller::srv::ControlChannel_Request>()
{
  return dcs103e_controller::srv::builder::Init_ControlChannel_Request_channel();
}

}  // namespace dcs103e_controller


namespace dcs103e_controller
{

namespace srv
{

namespace builder
{

class Init_ControlChannel_Response_message
{
public:
  explicit Init_ControlChannel_Response_message(::dcs103e_controller::srv::ControlChannel_Response & msg)
  : msg_(msg)
  {}
  ::dcs103e_controller::srv::ControlChannel_Response message(::dcs103e_controller::srv::ControlChannel_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dcs103e_controller::srv::ControlChannel_Response msg_;
};

class Init_ControlChannel_Response_success
{
public:
  Init_ControlChannel_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlChannel_Response_message success(::dcs103e_controller::srv::ControlChannel_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ControlChannel_Response_message(msg_);
  }

private:
  ::dcs103e_controller::srv::ControlChannel_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dcs103e_controller::srv::ControlChannel_Response>()
{
  return dcs103e_controller::srv::builder::Init_ControlChannel_Response_success();
}

}  // namespace dcs103e_controller

#endif  // DCS103E_CONTROLLER__SRV__DETAIL__CONTROL_CHANNEL__BUILDER_HPP_
