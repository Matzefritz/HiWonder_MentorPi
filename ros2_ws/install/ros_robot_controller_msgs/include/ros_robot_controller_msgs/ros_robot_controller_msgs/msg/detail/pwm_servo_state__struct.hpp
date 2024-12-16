// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_robot_controller_msgs:msg/PWMServoState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/pwm_servo_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__PWM_SERVO_STATE__STRUCT_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__PWM_SERVO_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros_robot_controller_msgs__msg__PWMServoState __attribute__((deprecated))
#else
# define DEPRECATED__ros_robot_controller_msgs__msg__PWMServoState __declspec(deprecated)
#endif

namespace ros_robot_controller_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PWMServoState_
{
  using Type = PWMServoState_<ContainerAllocator>;

  explicit PWMServoState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit PWMServoState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _id_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _id_type id;
  using _position_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _position_type position;
  using _offset_type =
    std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>>;
  _offset_type offset;

  // setters for named parameter idiom
  Type & set__id(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__position(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__offset(
    const std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>> & _arg)
  {
    this->offset = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_robot_controller_msgs__msg__PWMServoState
    std::shared_ptr<ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_robot_controller_msgs__msg__PWMServoState
    std::shared_ptr<ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PWMServoState_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->offset != other.offset) {
      return false;
    }
    return true;
  }
  bool operator!=(const PWMServoState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PWMServoState_

// alias to use template instance with default allocator
using PWMServoState =
  ros_robot_controller_msgs::msg::PWMServoState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__PWM_SERVO_STATE__STRUCT_HPP_
