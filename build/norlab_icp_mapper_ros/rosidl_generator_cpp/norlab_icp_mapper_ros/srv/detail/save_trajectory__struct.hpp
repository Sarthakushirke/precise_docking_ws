// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from norlab_icp_mapper_ros:srv/SaveTrajectory.idl
// generated code does not contain a copyright notice

#ifndef NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__SAVE_TRAJECTORY__STRUCT_HPP_
#define NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__SAVE_TRAJECTORY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'trajectory_file_name'
#include "std_msgs/msg/detail/string__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__norlab_icp_mapper_ros__srv__SaveTrajectory_Request __attribute__((deprecated))
#else
# define DEPRECATED__norlab_icp_mapper_ros__srv__SaveTrajectory_Request __declspec(deprecated)
#endif

namespace norlab_icp_mapper_ros
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SaveTrajectory_Request_
{
  using Type = SaveTrajectory_Request_<ContainerAllocator>;

  explicit SaveTrajectory_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : trajectory_file_name(_init)
  {
    (void)_init;
  }

  explicit SaveTrajectory_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : trajectory_file_name(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _trajectory_file_name_type =
    std_msgs::msg::String_<ContainerAllocator>;
  _trajectory_file_name_type trajectory_file_name;

  // setters for named parameter idiom
  Type & set__trajectory_file_name(
    const std_msgs::msg::String_<ContainerAllocator> & _arg)
  {
    this->trajectory_file_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    norlab_icp_mapper_ros::srv::SaveTrajectory_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const norlab_icp_mapper_ros::srv::SaveTrajectory_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      norlab_icp_mapper_ros::srv::SaveTrajectory_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      norlab_icp_mapper_ros::srv::SaveTrajectory_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__norlab_icp_mapper_ros__srv__SaveTrajectory_Request
    std::shared_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__norlab_icp_mapper_ros__srv__SaveTrajectory_Request
    std::shared_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SaveTrajectory_Request_ & other) const
  {
    if (this->trajectory_file_name != other.trajectory_file_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const SaveTrajectory_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SaveTrajectory_Request_

// alias to use template instance with default allocator
using SaveTrajectory_Request =
  norlab_icp_mapper_ros::srv::SaveTrajectory_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace norlab_icp_mapper_ros


#ifndef _WIN32
# define DEPRECATED__norlab_icp_mapper_ros__srv__SaveTrajectory_Response __attribute__((deprecated))
#else
# define DEPRECATED__norlab_icp_mapper_ros__srv__SaveTrajectory_Response __declspec(deprecated)
#endif

namespace norlab_icp_mapper_ros
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SaveTrajectory_Response_
{
  using Type = SaveTrajectory_Response_<ContainerAllocator>;

  explicit SaveTrajectory_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit SaveTrajectory_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    norlab_icp_mapper_ros::srv::SaveTrajectory_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const norlab_icp_mapper_ros::srv::SaveTrajectory_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      norlab_icp_mapper_ros::srv::SaveTrajectory_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      norlab_icp_mapper_ros::srv::SaveTrajectory_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__norlab_icp_mapper_ros__srv__SaveTrajectory_Response
    std::shared_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__norlab_icp_mapper_ros__srv__SaveTrajectory_Response
    std::shared_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SaveTrajectory_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const SaveTrajectory_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SaveTrajectory_Response_

// alias to use template instance with default allocator
using SaveTrajectory_Response =
  norlab_icp_mapper_ros::srv::SaveTrajectory_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace norlab_icp_mapper_ros

namespace norlab_icp_mapper_ros
{

namespace srv
{

struct SaveTrajectory
{
  using Request = norlab_icp_mapper_ros::srv::SaveTrajectory_Request;
  using Response = norlab_icp_mapper_ros::srv::SaveTrajectory_Response;
};

}  // namespace srv

}  // namespace norlab_icp_mapper_ros

#endif  // NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__SAVE_TRAJECTORY__STRUCT_HPP_
