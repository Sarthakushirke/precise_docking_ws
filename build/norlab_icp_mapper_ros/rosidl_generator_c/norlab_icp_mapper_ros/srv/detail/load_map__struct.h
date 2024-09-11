// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from norlab_icp_mapper_ros:srv/LoadMap.idl
// generated code does not contain a copyright notice

#ifndef NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__LOAD_MAP__STRUCT_H_
#define NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__LOAD_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'map_file_name'
#include "std_msgs/msg/detail/string__struct.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/LoadMap in the package norlab_icp_mapper_ros.
typedef struct norlab_icp_mapper_ros__srv__LoadMap_Request
{
  std_msgs__msg__String map_file_name;
  geometry_msgs__msg__Pose pose;
} norlab_icp_mapper_ros__srv__LoadMap_Request;

// Struct for a sequence of norlab_icp_mapper_ros__srv__LoadMap_Request.
typedef struct norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence
{
  norlab_icp_mapper_ros__srv__LoadMap_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/LoadMap in the package norlab_icp_mapper_ros.
typedef struct norlab_icp_mapper_ros__srv__LoadMap_Response
{
  uint8_t structure_needs_at_least_one_member;
} norlab_icp_mapper_ros__srv__LoadMap_Response;

// Struct for a sequence of norlab_icp_mapper_ros__srv__LoadMap_Response.
typedef struct norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence
{
  norlab_icp_mapper_ros__srv__LoadMap_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__LOAD_MAP__STRUCT_H_
