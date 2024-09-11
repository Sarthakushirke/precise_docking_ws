// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from norlab_icp_mapper_ros:srv/SaveTrajectory.idl
// generated code does not contain a copyright notice

#ifndef NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__SAVE_TRAJECTORY__STRUCT_H_
#define NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__SAVE_TRAJECTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'trajectory_file_name'
#include "std_msgs/msg/detail/string__struct.h"

/// Struct defined in srv/SaveTrajectory in the package norlab_icp_mapper_ros.
typedef struct norlab_icp_mapper_ros__srv__SaveTrajectory_Request
{
  std_msgs__msg__String trajectory_file_name;
} norlab_icp_mapper_ros__srv__SaveTrajectory_Request;

// Struct for a sequence of norlab_icp_mapper_ros__srv__SaveTrajectory_Request.
typedef struct norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence
{
  norlab_icp_mapper_ros__srv__SaveTrajectory_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} norlab_icp_mapper_ros__srv__SaveTrajectory_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SaveTrajectory in the package norlab_icp_mapper_ros.
typedef struct norlab_icp_mapper_ros__srv__SaveTrajectory_Response
{
  uint8_t structure_needs_at_least_one_member;
} norlab_icp_mapper_ros__srv__SaveTrajectory_Response;

// Struct for a sequence of norlab_icp_mapper_ros__srv__SaveTrajectory_Response.
typedef struct norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence
{
  norlab_icp_mapper_ros__srv__SaveTrajectory_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} norlab_icp_mapper_ros__srv__SaveTrajectory_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__SAVE_TRAJECTORY__STRUCT_H_
