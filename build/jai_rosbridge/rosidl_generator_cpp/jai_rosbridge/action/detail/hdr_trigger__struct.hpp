// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jai_rosbridge:action/HDRTrigger.idl
// generated code does not contain a copyright notice

#ifndef JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__STRUCT_HPP_
#define JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_Goal __attribute__((deprecated))
#else
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_Goal __declspec(deprecated)
#endif

namespace jai_rosbridge
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct HDRTrigger_Goal_
{
  using Type = HDRTrigger_Goal_<ContainerAllocator>;

  explicit HDRTrigger_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->space_id = "";
    }
  }

  explicit HDRTrigger_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : space_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->space_id = "";
    }
  }

  // field types and members
  using _space_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _space_id_type space_id;

  // setters for named parameter idiom
  Type & set__space_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->space_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jai_rosbridge::action::HDRTrigger_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const jai_rosbridge::action::HDRTrigger_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_Goal
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_Goal
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HDRTrigger_Goal_ & other) const
  {
    if (this->space_id != other.space_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const HDRTrigger_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HDRTrigger_Goal_

// alias to use template instance with default allocator
using HDRTrigger_Goal =
  jai_rosbridge::action::HDRTrigger_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace jai_rosbridge


#ifndef _WIN32
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_Result __attribute__((deprecated))
#else
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_Result __declspec(deprecated)
#endif

namespace jai_rosbridge
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct HDRTrigger_Result_
{
  using Type = HDRTrigger_Result_<ContainerAllocator>;

  explicit HDRTrigger_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result_message = "";
    }
  }

  explicit HDRTrigger_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result_message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _result_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _result_message_type result_message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__result_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->result_message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jai_rosbridge::action::HDRTrigger_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const jai_rosbridge::action::HDRTrigger_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_Result
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_Result
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HDRTrigger_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->result_message != other.result_message) {
      return false;
    }
    return true;
  }
  bool operator!=(const HDRTrigger_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HDRTrigger_Result_

// alias to use template instance with default allocator
using HDRTrigger_Result =
  jai_rosbridge::action::HDRTrigger_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace jai_rosbridge


#ifndef _WIN32
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_Feedback __declspec(deprecated)
#endif

namespace jai_rosbridge
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct HDRTrigger_Feedback_
{
  using Type = HDRTrigger_Feedback_<ContainerAllocator>;

  explicit HDRTrigger_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->feedback_message = "";
    }
  }

  explicit HDRTrigger_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : feedback_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->feedback_message = "";
    }
  }

  // field types and members
  using _feedback_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _feedback_message_type feedback_message;

  // setters for named parameter idiom
  Type & set__feedback_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->feedback_message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jai_rosbridge::action::HDRTrigger_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const jai_rosbridge::action::HDRTrigger_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_Feedback
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_Feedback
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HDRTrigger_Feedback_ & other) const
  {
    if (this->feedback_message != other.feedback_message) {
      return false;
    }
    return true;
  }
  bool operator!=(const HDRTrigger_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HDRTrigger_Feedback_

// alias to use template instance with default allocator
using HDRTrigger_Feedback =
  jai_rosbridge::action::HDRTrigger_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace jai_rosbridge


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "jai_rosbridge/action/detail/hdr_trigger__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_SendGoal_Request __declspec(deprecated)
#endif

namespace jai_rosbridge
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct HDRTrigger_SendGoal_Request_
{
  using Type = HDRTrigger_SendGoal_Request_<ContainerAllocator>;

  explicit HDRTrigger_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit HDRTrigger_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    jai_rosbridge::action::HDRTrigger_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const jai_rosbridge::action::HDRTrigger_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jai_rosbridge::action::HDRTrigger_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const jai_rosbridge::action::HDRTrigger_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_SendGoal_Request
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_SendGoal_Request
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HDRTrigger_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const HDRTrigger_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HDRTrigger_SendGoal_Request_

// alias to use template instance with default allocator
using HDRTrigger_SendGoal_Request =
  jai_rosbridge::action::HDRTrigger_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace jai_rosbridge


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_SendGoal_Response __declspec(deprecated)
#endif

namespace jai_rosbridge
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct HDRTrigger_SendGoal_Response_
{
  using Type = HDRTrigger_SendGoal_Response_<ContainerAllocator>;

  explicit HDRTrigger_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit HDRTrigger_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jai_rosbridge::action::HDRTrigger_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const jai_rosbridge::action::HDRTrigger_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_SendGoal_Response
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_SendGoal_Response
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HDRTrigger_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const HDRTrigger_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HDRTrigger_SendGoal_Response_

// alias to use template instance with default allocator
using HDRTrigger_SendGoal_Response =
  jai_rosbridge::action::HDRTrigger_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace jai_rosbridge

namespace jai_rosbridge
{

namespace action
{

struct HDRTrigger_SendGoal
{
  using Request = jai_rosbridge::action::HDRTrigger_SendGoal_Request;
  using Response = jai_rosbridge::action::HDRTrigger_SendGoal_Response;
};

}  // namespace action

}  // namespace jai_rosbridge


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_GetResult_Request __declspec(deprecated)
#endif

namespace jai_rosbridge
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct HDRTrigger_GetResult_Request_
{
  using Type = HDRTrigger_GetResult_Request_<ContainerAllocator>;

  explicit HDRTrigger_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit HDRTrigger_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jai_rosbridge::action::HDRTrigger_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const jai_rosbridge::action::HDRTrigger_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_GetResult_Request
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_GetResult_Request
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HDRTrigger_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const HDRTrigger_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HDRTrigger_GetResult_Request_

// alias to use template instance with default allocator
using HDRTrigger_GetResult_Request =
  jai_rosbridge::action::HDRTrigger_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace jai_rosbridge


// Include directives for member types
// Member 'result'
// already included above
// #include "jai_rosbridge/action/detail/hdr_trigger__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_GetResult_Response __declspec(deprecated)
#endif

namespace jai_rosbridge
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct HDRTrigger_GetResult_Response_
{
  using Type = HDRTrigger_GetResult_Response_<ContainerAllocator>;

  explicit HDRTrigger_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit HDRTrigger_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    jai_rosbridge::action::HDRTrigger_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const jai_rosbridge::action::HDRTrigger_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jai_rosbridge::action::HDRTrigger_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const jai_rosbridge::action::HDRTrigger_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_GetResult_Response
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_GetResult_Response
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HDRTrigger_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const HDRTrigger_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HDRTrigger_GetResult_Response_

// alias to use template instance with default allocator
using HDRTrigger_GetResult_Response =
  jai_rosbridge::action::HDRTrigger_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace jai_rosbridge

namespace jai_rosbridge
{

namespace action
{

struct HDRTrigger_GetResult
{
  using Request = jai_rosbridge::action::HDRTrigger_GetResult_Request;
  using Response = jai_rosbridge::action::HDRTrigger_GetResult_Response;
};

}  // namespace action

}  // namespace jai_rosbridge


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "jai_rosbridge/action/detail/hdr_trigger__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__jai_rosbridge__action__HDRTrigger_FeedbackMessage __declspec(deprecated)
#endif

namespace jai_rosbridge
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct HDRTrigger_FeedbackMessage_
{
  using Type = HDRTrigger_FeedbackMessage_<ContainerAllocator>;

  explicit HDRTrigger_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit HDRTrigger_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    jai_rosbridge::action::HDRTrigger_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const jai_rosbridge::action::HDRTrigger_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jai_rosbridge::action::HDRTrigger_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const jai_rosbridge::action::HDRTrigger_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jai_rosbridge::action::HDRTrigger_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jai_rosbridge::action::HDRTrigger_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jai_rosbridge::action::HDRTrigger_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_FeedbackMessage
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jai_rosbridge__action__HDRTrigger_FeedbackMessage
    std::shared_ptr<jai_rosbridge::action::HDRTrigger_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HDRTrigger_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const HDRTrigger_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HDRTrigger_FeedbackMessage_

// alias to use template instance with default allocator
using HDRTrigger_FeedbackMessage =
  jai_rosbridge::action::HDRTrigger_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace jai_rosbridge

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace jai_rosbridge
{

namespace action
{

struct HDRTrigger
{
  /// The goal message defined in the action definition.
  using Goal = jai_rosbridge::action::HDRTrigger_Goal;
  /// The result message defined in the action definition.
  using Result = jai_rosbridge::action::HDRTrigger_Result;
  /// The feedback message defined in the action definition.
  using Feedback = jai_rosbridge::action::HDRTrigger_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = jai_rosbridge::action::HDRTrigger_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = jai_rosbridge::action::HDRTrigger_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = jai_rosbridge::action::HDRTrigger_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct HDRTrigger HDRTrigger;

}  // namespace action

}  // namespace jai_rosbridge

#endif  // JAI_ROSBRIDGE__ACTION__DETAIL__HDR_TRIGGER__STRUCT_HPP_
