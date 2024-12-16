// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/ServosPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/servos_position.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SERVOS_POSITION__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SERVOS_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/servos_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_ServosPosition_position
{
public:
  explicit Init_ServosPosition_position(::ros_robot_controller_msgs::msg::ServosPosition & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::ServosPosition position(::ros_robot_controller_msgs::msg::ServosPosition::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::ServosPosition msg_;
};

class Init_ServosPosition_duration
{
public:
  Init_ServosPosition_duration()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ServosPosition_position duration(::ros_robot_controller_msgs::msg::ServosPosition::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return Init_ServosPosition_position(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::ServosPosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::ServosPosition>()
{
  return ros_robot_controller_msgs::msg::builder::Init_ServosPosition_duration();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SERVOS_POSITION__BUILDER_HPP_
