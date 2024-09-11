// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from norlab_icp_mapper_ros:srv/LoadMap.idl
// generated code does not contain a copyright notice

#ifndef NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__LOAD_MAP__TRAITS_HPP_
#define NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__LOAD_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "norlab_icp_mapper_ros/srv/detail/load_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'map_file_name'
#include "std_msgs/msg/detail/string__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace norlab_icp_mapper_ros
{

namespace srv
{

inline void to_flow_style_yaml(
  const LoadMap_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: map_file_name
  {
    out << "map_file_name: ";
    to_flow_style_yaml(msg.map_file_name, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LoadMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: map_file_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_file_name:\n";
    to_block_style_yaml(msg.map_file_name, out, indentation + 2);
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LoadMap_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace norlab_icp_mapper_ros

namespace rosidl_generator_traits
{

[[deprecated("use norlab_icp_mapper_ros::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const norlab_icp_mapper_ros::srv::LoadMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  norlab_icp_mapper_ros::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use norlab_icp_mapper_ros::srv::to_yaml() instead")]]
inline std::string to_yaml(const norlab_icp_mapper_ros::srv::LoadMap_Request & msg)
{
  return norlab_icp_mapper_ros::srv::to_yaml(msg);
}

template<>
inline const char * data_type<norlab_icp_mapper_ros::srv::LoadMap_Request>()
{
  return "norlab_icp_mapper_ros::srv::LoadMap_Request";
}

template<>
inline const char * name<norlab_icp_mapper_ros::srv::LoadMap_Request>()
{
  return "norlab_icp_mapper_ros/srv/LoadMap_Request";
}

template<>
struct has_fixed_size<norlab_icp_mapper_ros::srv::LoadMap_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<std_msgs::msg::String>::value> {};

template<>
struct has_bounded_size<norlab_icp_mapper_ros::srv::LoadMap_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<std_msgs::msg::String>::value> {};

template<>
struct is_message<norlab_icp_mapper_ros::srv::LoadMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace norlab_icp_mapper_ros
{

namespace srv
{

inline void to_flow_style_yaml(
  const LoadMap_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LoadMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LoadMap_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace norlab_icp_mapper_ros

namespace rosidl_generator_traits
{

[[deprecated("use norlab_icp_mapper_ros::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const norlab_icp_mapper_ros::srv::LoadMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  norlab_icp_mapper_ros::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use norlab_icp_mapper_ros::srv::to_yaml() instead")]]
inline std::string to_yaml(const norlab_icp_mapper_ros::srv::LoadMap_Response & msg)
{
  return norlab_icp_mapper_ros::srv::to_yaml(msg);
}

template<>
inline const char * data_type<norlab_icp_mapper_ros::srv::LoadMap_Response>()
{
  return "norlab_icp_mapper_ros::srv::LoadMap_Response";
}

template<>
inline const char * name<norlab_icp_mapper_ros::srv::LoadMap_Response>()
{
  return "norlab_icp_mapper_ros/srv/LoadMap_Response";
}

template<>
struct has_fixed_size<norlab_icp_mapper_ros::srv::LoadMap_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<norlab_icp_mapper_ros::srv::LoadMap_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<norlab_icp_mapper_ros::srv::LoadMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<norlab_icp_mapper_ros::srv::LoadMap>()
{
  return "norlab_icp_mapper_ros::srv::LoadMap";
}

template<>
inline const char * name<norlab_icp_mapper_ros::srv::LoadMap>()
{
  return "norlab_icp_mapper_ros/srv/LoadMap";
}

template<>
struct has_fixed_size<norlab_icp_mapper_ros::srv::LoadMap>
  : std::integral_constant<
    bool,
    has_fixed_size<norlab_icp_mapper_ros::srv::LoadMap_Request>::value &&
    has_fixed_size<norlab_icp_mapper_ros::srv::LoadMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<norlab_icp_mapper_ros::srv::LoadMap>
  : std::integral_constant<
    bool,
    has_bounded_size<norlab_icp_mapper_ros::srv::LoadMap_Request>::value &&
    has_bounded_size<norlab_icp_mapper_ros::srv::LoadMap_Response>::value
  >
{
};

template<>
struct is_service<norlab_icp_mapper_ros::srv::LoadMap>
  : std::true_type
{
};

template<>
struct is_service_request<norlab_icp_mapper_ros::srv::LoadMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<norlab_icp_mapper_ros::srv::LoadMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__LOAD_MAP__TRAITS_HPP_
