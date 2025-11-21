// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from piper_msgs:srv/AutoMotionToggle.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__AUTO_MOTION_TOGGLE__BUILDER_HPP_
#define PIPER_MSGS__SRV__DETAIL__AUTO_MOTION_TOGGLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "piper_msgs/srv/detail/auto_motion_toggle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace piper_msgs
{

namespace srv
{

namespace builder
{

class Init_AutoMotionToggle_Request_enable
{
public:
  Init_AutoMotionToggle_Request_enable()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::piper_msgs::srv::AutoMotionToggle_Request enable(::piper_msgs::srv::AutoMotionToggle_Request::_enable_type arg)
  {
    msg_.enable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::piper_msgs::srv::AutoMotionToggle_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::piper_msgs::srv::AutoMotionToggle_Request>()
{
  return piper_msgs::srv::builder::Init_AutoMotionToggle_Request_enable();
}

}  // namespace piper_msgs


namespace piper_msgs
{

namespace srv
{

namespace builder
{

class Init_AutoMotionToggle_Response_success
{
public:
  Init_AutoMotionToggle_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::piper_msgs::srv::AutoMotionToggle_Response success(::piper_msgs::srv::AutoMotionToggle_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::piper_msgs::srv::AutoMotionToggle_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::piper_msgs::srv::AutoMotionToggle_Response>()
{
  return piper_msgs::srv::builder::Init_AutoMotionToggle_Response_success();
}

}  // namespace piper_msgs

#endif  // PIPER_MSGS__SRV__DETAIL__AUTO_MOTION_TOGGLE__BUILDER_HPP_
