// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from norlab_icp_mapper_ros:srv/LoadMap.idl
// generated code does not contain a copyright notice

#ifndef NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__LOAD_MAP__FUNCTIONS_H_
#define NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__LOAD_MAP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "norlab_icp_mapper_ros/msg/rosidl_generator_c__visibility_control.h"

#include "norlab_icp_mapper_ros/srv/detail/load_map__struct.h"

/// Initialize srv/LoadMap message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * norlab_icp_mapper_ros__srv__LoadMap_Request
 * )) before or use
 * norlab_icp_mapper_ros__srv__LoadMap_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
bool
norlab_icp_mapper_ros__srv__LoadMap_Request__init(norlab_icp_mapper_ros__srv__LoadMap_Request * msg);

/// Finalize srv/LoadMap message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
void
norlab_icp_mapper_ros__srv__LoadMap_Request__fini(norlab_icp_mapper_ros__srv__LoadMap_Request * msg);

/// Create srv/LoadMap message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * norlab_icp_mapper_ros__srv__LoadMap_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
norlab_icp_mapper_ros__srv__LoadMap_Request *
norlab_icp_mapper_ros__srv__LoadMap_Request__create();

/// Destroy srv/LoadMap message.
/**
 * It calls
 * norlab_icp_mapper_ros__srv__LoadMap_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
void
norlab_icp_mapper_ros__srv__LoadMap_Request__destroy(norlab_icp_mapper_ros__srv__LoadMap_Request * msg);

/// Check for srv/LoadMap message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
bool
norlab_icp_mapper_ros__srv__LoadMap_Request__are_equal(const norlab_icp_mapper_ros__srv__LoadMap_Request * lhs, const norlab_icp_mapper_ros__srv__LoadMap_Request * rhs);

/// Copy a srv/LoadMap message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
bool
norlab_icp_mapper_ros__srv__LoadMap_Request__copy(
  const norlab_icp_mapper_ros__srv__LoadMap_Request * input,
  norlab_icp_mapper_ros__srv__LoadMap_Request * output);

/// Initialize array of srv/LoadMap messages.
/**
 * It allocates the memory for the number of elements and calls
 * norlab_icp_mapper_ros__srv__LoadMap_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
bool
norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence__init(norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence * array, size_t size);

/// Finalize array of srv/LoadMap messages.
/**
 * It calls
 * norlab_icp_mapper_ros__srv__LoadMap_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
void
norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence__fini(norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence * array);

/// Create array of srv/LoadMap messages.
/**
 * It allocates the memory for the array and calls
 * norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence *
norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence__create(size_t size);

/// Destroy array of srv/LoadMap messages.
/**
 * It calls
 * norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
void
norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence__destroy(norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence * array);

/// Check for srv/LoadMap message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
bool
norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence__are_equal(const norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence * lhs, const norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence * rhs);

/// Copy an array of srv/LoadMap messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
bool
norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence__copy(
  const norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence * input,
  norlab_icp_mapper_ros__srv__LoadMap_Request__Sequence * output);

/// Initialize srv/LoadMap message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * norlab_icp_mapper_ros__srv__LoadMap_Response
 * )) before or use
 * norlab_icp_mapper_ros__srv__LoadMap_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
bool
norlab_icp_mapper_ros__srv__LoadMap_Response__init(norlab_icp_mapper_ros__srv__LoadMap_Response * msg);

/// Finalize srv/LoadMap message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
void
norlab_icp_mapper_ros__srv__LoadMap_Response__fini(norlab_icp_mapper_ros__srv__LoadMap_Response * msg);

/// Create srv/LoadMap message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * norlab_icp_mapper_ros__srv__LoadMap_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
norlab_icp_mapper_ros__srv__LoadMap_Response *
norlab_icp_mapper_ros__srv__LoadMap_Response__create();

/// Destroy srv/LoadMap message.
/**
 * It calls
 * norlab_icp_mapper_ros__srv__LoadMap_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
void
norlab_icp_mapper_ros__srv__LoadMap_Response__destroy(norlab_icp_mapper_ros__srv__LoadMap_Response * msg);

/// Check for srv/LoadMap message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
bool
norlab_icp_mapper_ros__srv__LoadMap_Response__are_equal(const norlab_icp_mapper_ros__srv__LoadMap_Response * lhs, const norlab_icp_mapper_ros__srv__LoadMap_Response * rhs);

/// Copy a srv/LoadMap message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
bool
norlab_icp_mapper_ros__srv__LoadMap_Response__copy(
  const norlab_icp_mapper_ros__srv__LoadMap_Response * input,
  norlab_icp_mapper_ros__srv__LoadMap_Response * output);

/// Initialize array of srv/LoadMap messages.
/**
 * It allocates the memory for the number of elements and calls
 * norlab_icp_mapper_ros__srv__LoadMap_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
bool
norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence__init(norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence * array, size_t size);

/// Finalize array of srv/LoadMap messages.
/**
 * It calls
 * norlab_icp_mapper_ros__srv__LoadMap_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
void
norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence__fini(norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence * array);

/// Create array of srv/LoadMap messages.
/**
 * It allocates the memory for the array and calls
 * norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence *
norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence__create(size_t size);

/// Destroy array of srv/LoadMap messages.
/**
 * It calls
 * norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
void
norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence__destroy(norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence * array);

/// Check for srv/LoadMap message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
bool
norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence__are_equal(const norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence * lhs, const norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence * rhs);

/// Copy an array of srv/LoadMap messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_norlab_icp_mapper_ros
bool
norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence__copy(
  const norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence * input,
  norlab_icp_mapper_ros__srv__LoadMap_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // NORLAB_ICP_MAPPER_ROS__SRV__DETAIL__LOAD_MAP__FUNCTIONS_H_
