// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/ServoPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/servo_position.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SERVO_POSITION__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SERVO_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/servo_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_ServoPosition_position
{
public:
  explicit Init_ServoPosition_position(::ros_robot_controller_msgs::msg::ServoPosition & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::ServoPosition position(::ros_robot_controller_msgs::msg::ServoPosition::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::ServoPosition msg_;
};

class Init_ServoPosition_id
{
public:
  Init_ServoPosition_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ServoPosition_position id(::ros_robot_controller_msgs::msg::ServoPosition::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_ServoPosition_position(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::ServoPosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::ServoPosition>()
{
  return ros_robot_controller_msgs::msg::builder::Init_ServoPosition_id();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SERVO_POSITION__BUILDER_HPP_
