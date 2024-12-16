// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_robot_controller_msgs:msg/MotorsState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/motors_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTORS_STATE__STRUCT_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTORS_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'data'
#include "ros_robot_controller_msgs/msg/detail/motor_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_robot_controller_msgs__msg__MotorsState __attribute__((deprecated))
#else
# define DEPRECATED__ros_robot_controller_msgs__msg__MotorsState __declspec(deprecated)
#endif

namespace ros_robot_controller_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorsState_
{
  using Type = MotorsState_<ContainerAllocator>;

  explicit MotorsState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MotorsState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _data_type =
    std::vector<ros_robot_controller_msgs::msg::MotorState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros_robot_controller_msgs::msg::MotorState_<ContainerAllocator>>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::vector<ros_robot_controller_msgs::msg::MotorState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros_robot_controller_msgs::msg::MotorState_<ContainerAllocator>>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_robot_controller_msgs::msg::MotorsState_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_robot_controller_msgs::msg::MotorsState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_robot_controller_msgs::msg::MotorsState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_robot_controller_msgs::msg::MotorsState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_robot_controller_msgs::msg::MotorsState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_robot_controller_msgs::msg::MotorsState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_robot_controller_msgs::msg::MotorsState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_robot_controller_msgs::msg::MotorsState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_robot_controller_msgs::msg::MotorsState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_robot_controller_msgs::msg::MotorsState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_robot_controller_msgs__msg__MotorsState
    std::shared_ptr<ros_robot_controller_msgs::msg::MotorsState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_robot_controller_msgs__msg__MotorsState
    std::shared_ptr<ros_robot_controller_msgs::msg::MotorsState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorsState_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorsState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorsState_

// alias to use template instance with default allocator
using MotorsState =
  ros_robot_controller_msgs::msg::MotorsState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTORS_STATE__STRUCT_HPP_
