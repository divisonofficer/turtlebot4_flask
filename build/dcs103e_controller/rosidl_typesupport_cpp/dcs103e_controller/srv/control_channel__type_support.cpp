// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from dcs103e_controller:srv/ControlChannel.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "dcs103e_controller/srv/detail/control_channel__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace dcs103e_controller
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ControlChannel_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ControlChannel_Request_type_support_ids_t;

static const _ControlChannel_Request_type_support_ids_t _ControlChannel_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ControlChannel_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ControlChannel_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ControlChannel_Request_type_support_symbol_names_t _ControlChannel_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dcs103e_controller, srv, ControlChannel_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dcs103e_controller, srv, ControlChannel_Request)),
  }
};

typedef struct _ControlChannel_Request_type_support_data_t
{
  void * data[2];
} _ControlChannel_Request_type_support_data_t;

static _ControlChannel_Request_type_support_data_t _ControlChannel_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ControlChannel_Request_message_typesupport_map = {
  2,
  "dcs103e_controller",
  &_ControlChannel_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ControlChannel_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ControlChannel_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ControlChannel_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ControlChannel_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace dcs103e_controller

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dcs103e_controller::srv::ControlChannel_Request>()
{
  return &::dcs103e_controller::srv::rosidl_typesupport_cpp::ControlChannel_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, dcs103e_controller, srv, ControlChannel_Request)() {
  return get_message_type_support_handle<dcs103e_controller::srv::ControlChannel_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "dcs103e_controller/srv/detail/control_channel__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace dcs103e_controller
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ControlChannel_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ControlChannel_Response_type_support_ids_t;

static const _ControlChannel_Response_type_support_ids_t _ControlChannel_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ControlChannel_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ControlChannel_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ControlChannel_Response_type_support_symbol_names_t _ControlChannel_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dcs103e_controller, srv, ControlChannel_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dcs103e_controller, srv, ControlChannel_Response)),
  }
};

typedef struct _ControlChannel_Response_type_support_data_t
{
  void * data[2];
} _ControlChannel_Response_type_support_data_t;

static _ControlChannel_Response_type_support_data_t _ControlChannel_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ControlChannel_Response_message_typesupport_map = {
  2,
  "dcs103e_controller",
  &_ControlChannel_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ControlChannel_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ControlChannel_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ControlChannel_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ControlChannel_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace dcs103e_controller

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dcs103e_controller::srv::ControlChannel_Response>()
{
  return &::dcs103e_controller::srv::rosidl_typesupport_cpp::ControlChannel_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, dcs103e_controller, srv, ControlChannel_Response)() {
  return get_message_type_support_handle<dcs103e_controller::srv::ControlChannel_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "dcs103e_controller/srv/detail/control_channel__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace dcs103e_controller
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ControlChannel_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ControlChannel_type_support_ids_t;

static const _ControlChannel_type_support_ids_t _ControlChannel_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ControlChannel_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ControlChannel_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ControlChannel_type_support_symbol_names_t _ControlChannel_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dcs103e_controller, srv, ControlChannel)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dcs103e_controller, srv, ControlChannel)),
  }
};

typedef struct _ControlChannel_type_support_data_t
{
  void * data[2];
} _ControlChannel_type_support_data_t;

static _ControlChannel_type_support_data_t _ControlChannel_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ControlChannel_service_typesupport_map = {
  2,
  "dcs103e_controller",
  &_ControlChannel_service_typesupport_ids.typesupport_identifier[0],
  &_ControlChannel_service_typesupport_symbol_names.symbol_name[0],
  &_ControlChannel_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ControlChannel_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ControlChannel_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace dcs103e_controller

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<dcs103e_controller::srv::ControlChannel>()
{
  return &::dcs103e_controller::srv::rosidl_typesupport_cpp::ControlChannel_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, dcs103e_controller, srv, ControlChannel)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<dcs103e_controller::srv::ControlChannel>();
}

#ifdef __cplusplus
}
#endif
