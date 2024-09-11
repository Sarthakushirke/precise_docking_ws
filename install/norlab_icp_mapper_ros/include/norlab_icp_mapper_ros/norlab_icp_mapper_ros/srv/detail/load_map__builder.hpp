// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from norlab_icp_mapper_ros:srv/LoadMap.idl
// generated code does not contain a copyright notice

#ifndef NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__LOAD_MAP__BUILDER_HPP_
#define NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__LOAD_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "norlab_icp_mapper_ros/srv/detail/load_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace norlab_icp_mapper_ros
{

namespace srv
{

namespace builder
{

class Init_LoadMap_Request_pose
{
public:
  explicit Init_LoadMap_Request_pose(::norlab_icp_mapper_ros::srv::LoadMap_Request & msg)
  : msg_(msg)
  {}
  ::norlab_icp_mapper_ros::srv::LoadMap_Request pose(::norlab_icp_mapper_ros::srv::LoadMap_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::norlab_icp_mapper_ros::srv::LoadMap_Request msg_;
};

class Init_LoadMap_Request_map_file_name
{
public:
  Init_LoadMap_Request_map_file_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LoadMap_Request_pose map_file_name(::norlab_icp_mapper_ros::srv::LoadMap_Request::_map_file_name_type arg)
  {
    msg_.map_file_name = std::move(arg);
    return Init_LoadMap_Request_pose(msg_);
  }

private:
  ::norlab_icp_mapper_ros::srv::LoadMap_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::norlab_icp_mapper_ros::srv::LoadMap_Request>()
{
  return norlab_icp_mapper_ros::srv::builder::Init_LoadMap_Request_map_file_name();
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
auto build<::norlab_icp_mapper_ros::srv::LoadMap_Response>()
{
  return ::norlab_icp_mapper_ros::srv::LoadMap_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace norlab_icp_mapper_ros

#endif  // NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__LOAD_MAP__BUILDER_HPP_
