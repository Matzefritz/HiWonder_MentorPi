// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/Sbus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/sbus.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SBUS__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SBUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/sbus__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_Sbus_channel
{
public:
  explicit Init_Sbus_channel(::ros_robot_controller_msgs::msg::Sbus & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::Sbus channel(::ros_robot_controller_msgs::msg::Sbus::_channel_type arg)
  {
    msg_.channel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::Sbus msg_;
};

class Init_Sbus_header
{
public:
  Init_Sbus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sbus_channel header(::ros_robot_controller_msgs::msg::Sbus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Sbus_channel(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::Sbus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::Sbus>()
{
  return ros_robot_controller_msgs::msg::builder::Init_Sbus_header();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SBUS__BUILDER_HPP_
