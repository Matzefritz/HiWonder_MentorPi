// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/GetPWMServoCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/get_pwm_servo_cmd.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_PWM_SERVO_CMD__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_PWM_SERVO_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/get_pwm_servo_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_GetPWMServoCmd_get_offset
{
public:
  explicit Init_GetPWMServoCmd_get_offset(::ros_robot_controller_msgs::msg::GetPWMServoCmd & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::GetPWMServoCmd get_offset(::ros_robot_controller_msgs::msg::GetPWMServoCmd::_get_offset_type arg)
  {
    msg_.get_offset = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::GetPWMServoCmd msg_;
};

class Init_GetPWMServoCmd_get_position
{
public:
  explicit Init_GetPWMServoCmd_get_position(::ros_robot_controller_msgs::msg::GetPWMServoCmd & msg)
  : msg_(msg)
  {}
  Init_GetPWMServoCmd_get_offset get_position(::ros_robot_controller_msgs::msg::GetPWMServoCmd::_get_position_type arg)
  {
    msg_.get_position = std::move(arg);
    return Init_GetPWMServoCmd_get_offset(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::GetPWMServoCmd msg_;
};

class Init_GetPWMServoCmd_id
{
public:
  Init_GetPWMServoCmd_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPWMServoCmd_get_position id(::ros_robot_controller_msgs::msg::GetPWMServoCmd::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_GetPWMServoCmd_get_position(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::GetPWMServoCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::GetPWMServoCmd>()
{
  return ros_robot_controller_msgs::msg::builder::Init_GetPWMServoCmd_id();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_PWM_SERVO_CMD__BUILDER_HPP_
