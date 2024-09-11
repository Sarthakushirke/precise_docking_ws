// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from norlab_icp_mapper_ros:srv/SaveMap.idl
// generated code does not contain a copyright notice

#ifndef NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_
#define NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "norlab_icp_mapper_ros/srv/detail/save_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace norlab_icp_mapper_ros
{

namespace srv
{

namespace builder
{

class Init_SaveMap_Request_map_file_name
{
public:
  Init_SaveMap_Request_map_file_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::norlab_icp_mapper_ros::srv::SaveMap_Request map_file_name(::norlab_icp_mapper_ros::srv::SaveMap_Request::_map_file_name_type arg)
  {
    msg_.map_file_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::norlab_icp_mapper_ros::srv::SaveMap_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::norlab_icp_mapper_ros::srv::SaveMap_Request>()
{
  return norlab_icp_mapper_ros::srv::builder::Init_SaveMap_Request_map_file_name();
}

}  // namespace norlab_icp_mapper_ros


namespace norlab_icp_mapper_ros
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::norlab_icp_mapper_ros::srv::SaveMap_Response>()
{
  return ::norlab_icp_mapper_ros::srv::SaveMap_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace norlab_icp_mapper_ros

#endif  // NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_
