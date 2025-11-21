// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jai_rosbridge:action/HDRTrigger.idl
// generated code does not contain a copyright notice

#ifndef JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__BUILDER_HPP_
#define JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jai_rosbridge/action/detail/hdr_trigger__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jai_rosbridge
{

namespace action
{

namespace builder
{

class Init_HDRTrigger_Goal_space_id
{
public:
  Init_HDRTrigger_Goal_space_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::jai_rosbridge::action::HDRTrigger_Goal space_id(::jai_rosbridge::action::HDRTrigger_Goal::_space_id_type arg)
  {
    msg_.space_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jai_rosbridge::action::HDRTrigger_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jai_rosbridge::action::HDRTrigger_Goal>()
{
  return jai_rosbridge::action::builder::Init_HDRTrigger_Goal_space_id();
}

}  // namespace jai_rosbridge


namespace jai_rosbridge
{

namespace action
{

namespace builder
{

class Init_HDRTrigger_Result_result_message
{
public:
  explicit Init_HDRTrigger_Result_result_message(::jai_rosbridge::action::HDRTrigger_Result & msg)
  : msg_(msg)
  {}
  ::jai_rosbridge::action::HDRTrigger_Result result_message(::jai_rosbridge::action::HDRTrigger_Result::_result_message_type arg)
  {
    msg_.result_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jai_rosbridge::action::HDRTrigger_Result msg_;
};

class Init_HDRTrigger_Result_success
{
public:
  Init_HDRTrigger_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HDRTrigger_Result_result_message success(::jai_rosbridge::action::HDRTrigger_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_HDRTrigger_Result_result_message(msg_);
  }

private:
  ::jai_rosbridge::action::HDRTrigger_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jai_rosbridge::action::HDRTrigger_Result>()
{
  return jai_rosbridge::action::builder::Init_HDRTrigger_Result_success();
}

}  // namespace jai_rosbridge


namespace jai_rosbridge
{

namespace action
{

namespace builder
{

class Init_HDRTrigger_Feedback_feedback_message
{
public:
  Init_HDRTrigger_Feedback_feedback_message()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::jai_rosbridge::action::HDRTrigger_Feedback feedback_message(::jai_rosbridge::action::HDRTrigger_Feedback::_feedback_message_type arg)
  {
    msg_.feedback_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jai_rosbridge::action::HDRTrigger_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jai_rosbridge::action::HDRTrigger_Feedback>()
{
  return jai_rosbridge::action::builder::Init_HDRTrigger_Feedback_feedback_message();
}

}  // namespace jai_rosbridge


namespace jai_rosbridge
{

namespace action
{

namespace builder
{

class Init_HDRTrigger_SendGoal_Request_goal
{
public:
  explicit Init_HDRTrigger_SendGoal_Request_goal(::jai_rosbridge::action::HDRTrigger_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::jai_rosbridge::action::HDRTrigger_SendGoal_Request goal(::jai_rosbridge::action::HDRTrigger_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jai_rosbridge::action::HDRTrigger_SendGoal_Request msg_;
};

class Init_HDRTrigger_SendGoal_Request_goal_id
{
public:
  Init_HDRTrigger_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HDRTrigger_SendGoal_Request_goal goal_id(::jai_rosbridge::action::HDRTrigger_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_HDRTrigger_SendGoal_Request_goal(msg_);
  }

private:
  ::jai_rosbridge::action::HDRTrigger_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jai_rosbridge::action::HDRTrigger_SendGoal_Request>()
{
  return jai_rosbridge::action::builder::Init_HDRTrigger_SendGoal_Request_goal_id();
}

}  // namespace jai_rosbridge


namespace jai_rosbridge
{

namespace action
{

namespace builder
{

class Init_HDRTrigger_SendGoal_Response_stamp
{
public:
  explicit Init_HDRTrigger_SendGoal_Response_stamp(::jai_rosbridge::action::HDRTrigger_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::jai_rosbridge::action::HDRTrigger_SendGoal_Response stamp(::jai_rosbridge::action::HDRTrigger_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jai_rosbridge::action::HDRTrigger_SendGoal_Response msg_;
};

class Init_HDRTrigger_SendGoal_Response_accepted
{
public:
  Init_HDRTrigger_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HDRTrigger_SendGoal_Response_stamp accepted(::jai_rosbridge::action::HDRTrigger_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_HDRTrigger_SendGoal_Response_stamp(msg_);
  }

private:
  ::jai_rosbridge::action::HDRTrigger_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jai_rosbridge::action::HDRTrigger_SendGoal_Response>()
{
  return jai_rosbridge::action::builder::Init_HDRTrigger_SendGoal_Response_accepted();
}

}  // namespace jai_rosbridge


namespace jai_rosbridge
{

namespace action
{

namespace builder
{

class Init_HDRTrigger_GetResult_Request_goal_id
{
public:
  Init_HDRTrigger_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::jai_rosbridge::action::HDRTrigger_GetResult_Request goal_id(::jai_rosbridge::action::HDRTrigger_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jai_rosbridge::action::HDRTrigger_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jai_rosbridge::action::HDRTrigger_GetResult_Request>()
{
  return jai_rosbridge::action::builder::Init_HDRTrigger_GetResult_Request_goal_id();
}

}  // namespace jai_rosbridge


namespace jai_rosbridge
{

namespace action
{

namespace builder
{

class Init_HDRTrigger_GetResult_Response_result
{
public:
  explicit Init_HDRTrigger_GetResult_Response_result(::jai_rosbridge::action::HDRTrigger_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::jai_rosbridge::action::HDRTrigger_GetResult_Response result(::jai_rosbridge::action::HDRTrigger_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jai_rosbridge::action::HDRTrigger_GetResult_Response msg_;
};

class Init_HDRTrigger_GetResult_Response_status
{
public:
  Init_HDRTrigger_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HDRTrigger_GetResult_Response_result status(::jai_rosbridge::action::HDRTrigger_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_HDRTrigger_GetResult_Response_result(msg_);
  }

private:
  ::jai_rosbridge::action::HDRTrigger_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jai_rosbridge::action::HDRTrigger_GetResult_Response>()
{
  return jai_rosbridge::action::builder::Init_HDRTrigger_GetResult_Response_status();
}

}  // namespace jai_rosbridge


namespace jai_rosbridge
{

namespace action
{

namespace builder
{

class Init_HDRTrigger_FeedbackMessage_feedback
{
public:
  explicit Init_HDRTrigger_FeedbackMessage_feedback(::jai_rosbridge::action::HDRTrigger_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::jai_rosbridge::action::HDRTrigger_FeedbackMessage feedback(::jai_rosbridge::action::HDRTrigger_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jai_rosbridge::action::HDRTrigger_FeedbackMessage msg_;
};

class Init_HDRTrigger_FeedbackMessage_goal_id
{
public:
  Init_HDRTrigger_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HDRTrigger_FeedbackMessage_feedback goal_id(::jai_rosbridge::action::HDRTrigger_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_HDRTrigger_FeedbackMessage_feedback(msg_);
  }

private:
  ::jai_rosbridge::action::HDRTrigger_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jai_rosbridge::action::HDRTrigger_FeedbackMessage>()
{
  return jai_rosbridge::action::builder::Init_HDRTrigger_FeedbackMessage_goal_id();
}

}  // namespace jai_rosbridge

#endif  // JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__BUILDER_HPP_
