// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/motor_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/motor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorState_rps
{
public:
  explicit Init_MotorState_rps(::ros_robot_controller_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::MotorState rps(::ros_robot_controller_msgs::msg::MotorState::_rps_type arg)
  {
    msg_.rps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::MotorState msg_;
};

class Init_MotorState_id
{
public:
  Init_MotorState_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorState_rps id(::ros_robot_controller_msgs::msg::MotorState::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_MotorState_rps(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::MotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::MotorState>()
{
  return ros_robot_controller_msgs::msg::builder::Init_MotorState_id();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
