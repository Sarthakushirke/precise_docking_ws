// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from norlab_icp_mapper_ros:srv/SaveTrajectory.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "norlab_icp_mapper_ros/srv/detail/save_trajectory__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace norlab_icp_mapper_ros
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SaveTrajectory_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) norlab_icp_mapper_ros::srv::SaveTrajectory_Request(_init);
}

void SaveTrajectory_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<norlab_icp_mapper_ros::srv::SaveTrajectory_Request *>(message_memory);
  typed_message->~SaveTrajectory_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SaveTrajectory_Request_message_member_array[1] = {
  {
    "trajectory_file_name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::String>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(norlab_icp_mapper_ros::srv::SaveTrajectory_Request, trajectory_file_name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SaveTrajectory_Request_message_members = {
  "norlab_icp_mapper_ros::srv",  // message namespace
  "SaveTrajectory_Request",  // message name
  1,  // number of fields
  sizeof(norlab_icp_mapper_ros::srv::SaveTrajectory_Request),
  SaveTrajectory_Request_message_member_array,  // message members
  SaveTrajectory_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SaveTrajectory_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SaveTrajectory_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SaveTrajectory_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace norlab_icp_mapper_ros


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<norlab_icp_mapper_ros::srv::SaveTrajectory_Request>()
{
  return &::norlab_icp_mapper_ros::srv::rosidl_typesupport_introspection_cpp::SaveTrajectory_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, norlab_icp_mapper_ros, srv, SaveTrajectory_Request)() {
  return &::norlab_icp_mapper_ros::srv::rosidl_typesupport_introspection_cpp::SaveTrajectory_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "norlab_icp_mapper_ros/srv/detail/save_trajectory__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace norlab_icp_mapper_ros
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SaveTrajectory_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) norlab_icp_mapper_ros::srv::SaveTrajectory_Response(_init);
}

void SaveTrajectory_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<norlab_icp_mapper_ros::srv::SaveTrajectory_Response *>(message_memory);
  typed_message->~SaveTrajectory_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SaveTrajectory_Response_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(norlab_icp_mapper_ros::srv::SaveTrajectory_Response, structure_needs_at_least_one_member),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SaveTrajectory_Response_message_members = {
  "norlab_icp_mapper_ros::srv",  // message namespace
  "SaveTrajectory_Response",  // message name
  1,  // number of fields
  sizeof(norlab_icp_mapper_ros::srv::SaveTrajectory_Response),
  SaveTrajectory_Response_message_member_array,  // message members
  SaveTrajectory_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SaveTrajectory_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SaveTrajectory_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SaveTrajectory_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace norlab_icp_mapper_ros


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<norlab_icp_mapper_ros::srv::SaveTrajectory_Response>()
{
  return &::norlab_icp_mapper_ros::srv::rosidl_typesupport_introspection_cpp::SaveTrajectory_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, norlab_icp_mapper_ros, srv, SaveTrajectory_Response)() {
  return &::norlab_icp_mapper_ros::srv::rosidl_typesupport_introspection_cpp::SaveTrajectory_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "norlab_icp_mapper_ros/srv/detail/save_trajectory__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace norlab_icp_mapper_ros
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers SaveTrajectory_service_members = {
  "norlab_icp_mapper_ros::srv",  // service namespace
  "SaveTrajectory",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<norlab_icp_mapper_ros::srv::SaveTrajectory>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t SaveTrajectory_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SaveTrajectory_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace norlab_icp_mapper_ros


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<norlab_icp_mapper_ros::srv::SaveTrajectory>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::norlab_icp_mapper_ros::srv::rosidl_typesupport_introspection_cpp::SaveTrajectory_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::norlab_icp_mapper_ros::srv::SaveTrajectory_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::norlab_icp_mapper_ros::srv::SaveTrajectory_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, norlab_icp_mapper_ros, srv, SaveTrajectory)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<norlab_icp_mapper_ros::srv::SaveTrajectory>();
}

#ifdef __cplusplus
}
#endif
