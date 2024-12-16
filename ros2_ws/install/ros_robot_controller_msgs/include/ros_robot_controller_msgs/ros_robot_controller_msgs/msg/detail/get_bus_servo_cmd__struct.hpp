// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_robot_controller_msgs:msg/GetBusServoCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/get_bus_servo_cmd.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_BUS_SERVO_CMD__STRUCT_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_BUS_SERVO_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros_robot_controller_msgs__msg__GetBusServoCmd __attribute__((deprecated))
#else
# define DEPRECATED__ros_robot_controller_msgs__msg__GetBusServoCmd __declspec(deprecated)
#endif

namespace ros_robot_controller_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GetBusServoCmd_
{
  using Type = GetBusServoCmd_<ContainerAllocator>;

  explicit GetBusServoCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->get_id = 0;
      this->get_position = 0;
      this->get_offset = 0;
      this->get_voltage = 0;
      this->get_temperature = 0;
      this->get_position_limit = 0;
      this->get_voltage_limit = 0;
      this->get_max_temperature_limit = 0;
      this->get_torque_state = 0;
    }
  }

  explicit GetBusServoCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->get_id = 0;
      this->get_position = 0;
      this->get_offset = 0;
      this->get_voltage = 0;
      this->get_temperature = 0;
      this->get_position_limit = 0;
      this->get_voltage_limit = 0;
      this->get_max_temperature_limit = 0;
      this->get_torque_state = 0;
    }
  }

  // field types and members
  using _id_type =
    uint8_t;
  _id_type id;
  using _get_id_type =
    uint8_t;
  _get_id_type get_id;
  using _get_position_type =
    uint8_t;
  _get_position_type get_position;
  using _get_offset_type =
    uint8_t;
  _get_offset_type get_offset;
  using _get_voltage_type =
    uint8_t;
  _get_voltage_type get_voltage;
  using _get_temperature_type =
    uint8_t;
  _get_temperature_type get_temperature;
  using _get_position_limit_type =
    uint8_t;
  _get_position_limit_type get_position_limit;
  using _get_voltage_limit_type =
    uint8_t;
  _get_voltage_limit_type get_voltage_limit;
  using _get_max_temperature_limit_type =
    uint8_t;
  _get_max_temperature_limit_type get_max_temperature_limit;
  using _get_torque_state_type =
    uint8_t;
  _get_torque_state_type get_torque_state;

  // setters for named parameter idiom
  Type & set__id(
    const uint8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__get_id(
    const uint8_t & _arg)
  {
    this->get_id = _arg;
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
  Type & set__get_voltage(
    const uint8_t & _arg)
  {
    this->get_voltage = _arg;
    return *this;
  }
  Type & set__get_temperature(
    const uint8_t & _arg)
  {
    this->get_temperature = _arg;
    return *this;
  }
  Type & set__get_position_limit(
    const uint8_t & _arg)
  {
    this->get_position_limit = _arg;
    return *this;
  }
  Type & set__get_voltage_limit(
    const uint8_t & _arg)
  {
    this->get_voltage_limit = _arg;
    return *this;
  }
  Type & set__get_max_temperature_limit(
    const uint8_t & _arg)
  {
    this->get_max_temperature_limit = _arg;
    return *this;
  }
  Type & set__get_torque_state(
    const uint8_t & _arg)
  {
    this->get_torque_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_robot_controller_msgs::msg::GetBusServoCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_robot_controller_msgs::msg::GetBusServoCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_robot_controller_msgs::msg::GetBusServoCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_robot_controller_msgs::msg::GetBusServoCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_robot_controller_msgs::msg::GetBusServoCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_robot_controller_msgs::msg::GetBusServoCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_robot_controller_msgs::msg::GetBusServoCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_robot_controller_msgs::msg::GetBusServoCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_robot_controller_msgs::msg::GetBusServoCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_robot_controller_msgs::msg::GetBusServoCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_robot_controller_msgs__msg__GetBusServoCmd
    std::shared_ptr<ros_robot_controller_msgs::msg::GetBusServoCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_robot_controller_msgs__msg__GetBusServoCmd
    std::shared_ptr<ros_robot_controller_msgs::msg::GetBusServoCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetBusServoCmd_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->get_id != other.get_id) {
      return false;
    }
    if (this->get_position != other.get_position) {
      return false;
    }
    if (this->get_offset != other.get_offset) {
      return false;
    }
    if (this->get_voltage != other.get_voltage) {
      return false;
    }
    if (this->get_temperature != other.get_temperature) {
      return false;
    }
    if (this->get_position_limit != other.get_position_limit) {
      return false;
    }
    if (this->get_voltage_limit != other.get_voltage_limit) {
      return false;
    }
    if (this->get_max_temperature_limit != other.get_max_temperature_limit) {
      return false;
    }
    if (this->get_torque_state != other.get_torque_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetBusServoCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetBusServoCmd_

// alias to use template instance with default allocator
using GetBusServoCmd =
  ros_robot_controller_msgs::msg::GetBusServoCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_BUS_SERVO_CMD__STRUCT_HPP_
