// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dcs103e_controller:srv/ControlChannel.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dcs103e_controller/srv/detail/control_channel__rosidl_typesupport_introspection_c.h"
#include "dcs103e_controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dcs103e_controller/srv/detail/control_channel__functions.h"
#include "dcs103e_controller/srv/detail/control_channel__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void dcs103e_controller__srv__ControlChannel_Request__rosidl_typesupport_introspection_c__ControlChannel_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dcs103e_controller__srv__ControlChannel_Request__init(message_memory);
}

void dcs103e_controller__srv__ControlChannel_Request__rosidl_typesupport_introspection_c__ControlChannel_Request_fini_function(void * message_memory)
{
  dcs103e_controller__srv__ControlChannel_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dcs103e_controller__srv__ControlChannel_Request__rosidl_typesupport_introspection_c__ControlChannel_Request_message_member_array[3] = {
  {
    "channel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dcs103e_controller__srv__ControlChannel_Request, channel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enable",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dcs103e_controller__srv__ControlChannel_Request, enable),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dcs103e_controller__srv__ControlChannel_Request, current),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dcs103e_controller__srv__ControlChannel_Request__rosidl_typesupport_introspection_c__ControlChannel_Request_message_members = {
  "dcs103e_controller__srv",  // message namespace
  "ControlChannel_Request",  // message name
  3,  // number of fields
  sizeof(dcs103e_controller__srv__ControlChannel_Request),
  dcs103e_controller__srv__ControlChannel_Request__rosidl_typesupport_introspection_c__ControlChannel_Request_message_member_array,  // message members
  dcs103e_controller__srv__ControlChannel_Request__rosidl_typesupport_introspection_c__ControlChannel_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  dcs103e_controller__srv__ControlChannel_Request__rosidl_typesupport_introspection_c__ControlChannel_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dcs103e_controller__srv__ControlChannel_Request__rosidl_typesupport_introspection_c__ControlChannel_Request_message_type_support_handle = {
  0,
  &dcs103e_controller__srv__ControlChannel_Request__rosidl_typesupport_introspection_c__ControlChannel_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dcs103e_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dcs103e_controller, srv, ControlChannel_Request)() {
  if (!dcs103e_controller__srv__ControlChannel_Request__rosidl_typesupport_introspection_c__ControlChannel_Request_message_type_support_handle.typesupport_identifier) {
    dcs103e_controller__srv__ControlChannel_Request__rosidl_typesupport_introspection_c__ControlChannel_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dcs103e_controller__srv__ControlChannel_Request__rosidl_typesupport_introspection_c__ControlChannel_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "dcs103e_controller/srv/detail/control_channel__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dcs103e_controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dcs103e_controller/srv/detail/control_channel__functions.h"
// already included above
// #include "dcs103e_controller/srv/detail/control_channel__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dcs103e_controller__srv__ControlChannel_Response__rosidl_typesupport_introspection_c__ControlChannel_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dcs103e_controller__srv__ControlChannel_Response__init(message_memory);
}

void dcs103e_controller__srv__ControlChannel_Response__rosidl_typesupport_introspection_c__ControlChannel_Response_fini_function(void * message_memory)
{
  dcs103e_controller__srv__ControlChannel_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dcs103e_controller__srv__ControlChannel_Response__rosidl_typesupport_introspection_c__ControlChannel_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dcs103e_controller__srv__ControlChannel_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dcs103e_controller__srv__ControlChannel_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dcs103e_controller__srv__ControlChannel_Response__rosidl_typesupport_introspection_c__ControlChannel_Response_message_members = {
  "dcs103e_controller__srv",  // message namespace
  "ControlChannel_Response",  // message name
  2,  // number of fields
  sizeof(dcs103e_controller__srv__ControlChannel_Response),
  dcs103e_controller__srv__ControlChannel_Response__rosidl_typesupport_introspection_c__ControlChannel_Response_message_member_array,  // message members
  dcs103e_controller__srv__ControlChannel_Response__rosidl_typesupport_introspection_c__ControlChannel_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  dcs103e_controller__srv__ControlChannel_Response__rosidl_typesupport_introspection_c__ControlChannel_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dcs103e_controller__srv__ControlChannel_Response__rosidl_typesupport_introspection_c__ControlChannel_Response_message_type_support_handle = {
  0,
  &dcs103e_controller__srv__ControlChannel_Response__rosidl_typesupport_introspection_c__ControlChannel_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dcs103e_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dcs103e_controller, srv, ControlChannel_Response)() {
  if (!dcs103e_controller__srv__ControlChannel_Response__rosidl_typesupport_introspection_c__ControlChannel_Response_message_type_support_handle.typesupport_identifier) {
    dcs103e_controller__srv__ControlChannel_Response__rosidl_typesupport_introspection_c__ControlChannel_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dcs103e_controller__srv__ControlChannel_Response__rosidl_typesupport_introspection_c__ControlChannel_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "dcs103e_controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "dcs103e_controller/srv/detail/control_channel__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers dcs103e_controller__srv__detail__control_channel__rosidl_typesupport_introspection_c__ControlChannel_service_members = {
  "dcs103e_controller__srv",  // service namespace
  "ControlChannel",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // dcs103e_controller__srv__detail__control_channel__rosidl_typesupport_introspection_c__ControlChannel_Request_message_type_support_handle,
  NULL  // response message
  // dcs103e_controller__srv__detail__control_channel__rosidl_typesupport_introspection_c__ControlChannel_Response_message_type_support_handle
};

static rosidl_service_type_support_t dcs103e_controller__srv__detail__control_channel__rosidl_typesupport_introspection_c__ControlChannel_service_type_support_handle = {
  0,
  &dcs103e_controller__srv__detail__control_channel__rosidl_typesupport_introspection_c__ControlChannel_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dcs103e_controller, srv, ControlChannel_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dcs103e_controller, srv, ControlChannel_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dcs103e_controller
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dcs103e_controller, srv, ControlChannel)() {
  if (!dcs103e_controller__srv__detail__control_channel__rosidl_typesupport_introspection_c__ControlChannel_service_type_support_handle.typesupport_identifier) {
    dcs103e_controller__srv__detail__control_channel__rosidl_typesupport_introspection_c__ControlChannel_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)dcs103e_controller__srv__detail__control_channel__rosidl_typesupport_introspection_c__ControlChannel_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dcs103e_controller, srv, ControlChannel_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dcs103e_controller, srv, ControlChannel_Response)()->data;
  }

  return &dcs103e_controller__srv__detail__control_channel__rosidl_typesupport_introspection_c__ControlChannel_service_type_support_handle;
}
