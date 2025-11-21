// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from piper_msgs:srv/MoveArm.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__MOVE_ARM__BUILDER_HPP_
#define PIPER_MSGS__SRV__DETAIL__MOVE_ARM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "piper_msgs/srv/detail/move_arm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace piper_msgs
{

namespace srv
{

namespace builder
{

class Init_MoveArm_Request_move_idx
{
public:
  Init_MoveArm_Request_move_idx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::piper_msgs::srv::MoveArm_Request move_idx(::piper_msgs::srv::MoveArm_Request::_move_idx_type arg)
  {
    msg_.move_idx = std::move(arg);
    return std::move(msg_);
  }

private:
  ::piper_msgs::srv::MoveArm_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::piper_msgs::srv::MoveArm_Request>()
{
  return piper_msgs::srv::builder::Init_MoveArm_Request_move_idx();
}

}  // namespace piper_msgs


namespace piper_msgs
{

namespace srv
{

namespace builder
{

class Init_MoveArm_Response_success
{
public:
  Init_MoveArm_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::piper_msgs::srv::MoveArm_Response success(::piper_msgs::srv::MoveArm_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::piper_msgs::srv::MoveArm_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::piper_msgs::srv::MoveArm_Response>()
{
  return piper_msgs::srv::builder::Init_MoveArm_Response_success();
}

}  // namespace piper_msgs

#endif  // PIPER_MSGS__SRV__DETAIL__MOVE_ARM__BUILDER_HPP_
