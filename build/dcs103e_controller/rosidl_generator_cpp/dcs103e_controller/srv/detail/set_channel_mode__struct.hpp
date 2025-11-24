// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dcs103e_controller:srv/SetChannelMode.idl
// generated code does not contain a copyright notice

#ifndef DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__STRUCT_HPP_
#define DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dcs103e_controller__srv__SetChannelMode_Request __attribute__((deprecated))
#else
# define DEPRECATED__dcs103e_controller__srv__SetChannelMode_Request __declspec(deprecated)
#endif

namespace dcs103e_controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetChannelMode_Request_
{
  using Type = SetChannelMode_Request_<ContainerAllocator>;

  explicit SetChannelMode_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->channel = 0l;
      this->mode = "";
      this->current = 0.0;
      this->pulse_width = 0.0;
      this->pulse_delay = 0.0;
    }
  }

  explicit SetChannelMode_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mode(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->channel = 0l;
      this->mode = "";
      this->current = 0.0;
      this->pulse_width = 0.0;
      this->pulse_delay = 0.0;
    }
  }

  // field types and members
  using _channel_type =
    int32_t;
  _channel_type channel;
  using _mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mode_type mode;
  using _current_type =
    double;
  _current_type current;
  using _pulse_width_type =
    double;
  _pulse_width_type pulse_width;
  using _pulse_delay_type =
    double;
  _pulse_delay_type pulse_delay;

  // setters for named parameter idiom
  Type & set__channel(
    const int32_t & _arg)
  {
    this->channel = _arg;
    return *this;
  }
  Type & set__mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__current(
    const double & _arg)
  {
    this->current = _arg;
    return *this;
  }
  Type & set__pulse_width(
    const double & _arg)
  {
    this->pulse_width = _arg;
    return *this;
  }
  Type & set__pulse_delay(
    const double & _arg)
  {
    this->pulse_delay = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dcs103e_controller::srv::SetChannelMode_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dcs103e_controller::srv::SetChannelMode_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dcs103e_controller::srv::SetChannelMode_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dcs103e_controller::srv::SetChannelMode_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dcs103e_controller::srv::SetChannelMode_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dcs103e_controller::srv::SetChannelMode_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dcs103e_controller::srv::SetChannelMode_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dcs103e_controller::srv::SetChannelMode_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dcs103e_controller::srv::SetChannelMode_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dcs103e_controller::srv::SetChannelMode_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dcs103e_controller__srv__SetChannelMode_Request
    std::shared_ptr<dcs103e_controller::srv::SetChannelMode_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dcs103e_controller__srv__SetChannelMode_Request
    std::shared_ptr<dcs103e_controller::srv::SetChannelMode_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetChannelMode_Request_ & other) const
  {
    if (this->channel != other.channel) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->current != other.current) {
      return false;
    }
    if (this->pulse_width != other.pulse_width) {
      return false;
    }
    if (this->pulse_delay != other.pulse_delay) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetChannelMode_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetChannelMode_Request_

// alias to use template instance with default allocator
using SetChannelMode_Request =
  dcs103e_controller::srv::SetChannelMode_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dcs103e_controller


#ifndef _WIN32
# define DEPRECATED__dcs103e_controller__srv__SetChannelMode_Response __attribute__((deprecated))
#else
# define DEPRECATED__dcs103e_controller__srv__SetChannelMode_Response __declspec(deprecated)
#endif

namespace dcs103e_controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetChannelMode_Response_
{
  using Type = SetChannelMode_Response_<ContainerAllocator>;

  explicit SetChannelMode_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetChannelMode_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dcs103e_controller::srv::SetChannelMode_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dcs103e_controller::srv::SetChannelMode_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dcs103e_controller::srv::SetChannelMode_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dcs103e_controller::srv::SetChannelMode_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dcs103e_controller::srv::SetChannelMode_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dcs103e_controller::srv::SetChannelMode_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dcs103e_controller::srv::SetChannelMode_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dcs103e_controller::srv::SetChannelMode_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dcs103e_controller::srv::SetChannelMode_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dcs103e_controller::srv::SetChannelMode_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dcs103e_controller__srv__SetChannelMode_Response
    std::shared_ptr<dcs103e_controller::srv::SetChannelMode_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dcs103e_controller__srv__SetChannelMode_Response
    std::shared_ptr<dcs103e_controller::srv::SetChannelMode_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetChannelMode_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetChannelMode_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetChannelMode_Response_

// alias to use template instance with default allocator
using SetChannelMode_Response =
  dcs103e_controller::srv::SetChannelMode_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dcs103e_controller

namespace dcs103e_controller
{

namespace srv
{

struct SetChannelMode
{
  using Request = dcs103e_controller::srv::SetChannelMode_Request;
  using Response = dcs103e_controller::srv::SetChannelMode_Response;
};

}  // namespace srv

}  // namespace dcs103e_controller

#endif  // DCS103E_CONTROLLER__SRV__DETAIL__SET_CHANNEL_MODE__STRUCT_HPP_
