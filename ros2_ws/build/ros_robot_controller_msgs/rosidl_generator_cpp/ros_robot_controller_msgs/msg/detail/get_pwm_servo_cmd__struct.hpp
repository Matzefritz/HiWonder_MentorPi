// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_robot_controller_msgs:msg/GetPWMServoCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/get_pwm_servo_cmd.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_PWM_SERVO_CMD__STRUCT_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_PWM_SERVO_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros_robot_controller_msgs__msg__GetPWMServoCmd __attribute__((deprecated))
#else
# define DEPRECATED__ros_robot_controller_msgs__msg__GetPWMServoCmd __declspec(deprecated)
#endif

namespace ros_robot_controller_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GetPWMServoCmd_
{
  using Type = GetPWMServoCmd_<ContainerAllocator>;

  explicit GetPWMServoCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->get_position = 0;
      this->get_offset = 0;
    }
  }

  explicit GetPWMServoCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->get_position = 0;
      this->get_offset = 0;
    }
  }

  // field types and members
  using _id_type =
    uint8_t;
  _id_type id;
  using _get_position_type =
    uint8_t;
  _get_position_type get_position;
  using _get_offset_type =
    uint8_t;
  _get_offset_type get_offset;

  // setters for named parameter idiom
  Type & set__id(
    const uint8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__get_position(
    const uint8_t & _arg)
  {
    this->get_position = _arg;
    return *this;
  }
  Type & set__get_offset(
    const uint8_t & _arg)
  {
    this->get_offset = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_robot_controller_msgs__msg__GetPWMServoCmd
    std::shared_ptr<ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_robot_controller_msgs__msg__GetPWMServoCmd
    std::shared_ptr<ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPWMServoCmd_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->get_position != other.get_position) {
      return false;
    }
    if (this->get_offset != other.get_offset) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPWMServoCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPWMServoCmd_

// alias to use template instance with default allocator
using GetPWMServoCmd =
  ros_robot_controller_msgs::msg::GetPWMServoCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_PWM_SERVO_CMD__STRUCT_HPP_
