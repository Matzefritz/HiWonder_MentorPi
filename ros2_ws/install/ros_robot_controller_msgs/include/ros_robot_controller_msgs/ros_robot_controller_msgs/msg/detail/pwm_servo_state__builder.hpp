// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/PWMServoState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/pwm_servo_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__PWM_SERVO_STATE__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__PWM_SERVO_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/pwm_servo_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_PWMServoState_offset
{
public:
  explicit Init_PWMServoState_offset(::ros_robot_controller_msgs::msg::PWMServoState & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::PWMServoState offset(::ros_robot_controller_msgs::msg::PWMServoState::_offset_type arg)
  {
    msg_.offset = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::PWMServoState msg_;
};

class Init_PWMServoState_position
{
public:
  explicit Init_PWMServoState_position(::ros_robot_controller_msgs::msg::PWMServoState & msg)
  : msg_(msg)
  {}
  Init_PWMServoState_offset position(::ros_robot_controller_msgs::msg::PWMServoState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_PWMServoState_offset(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::PWMServoState msg_;
};

class Init_PWMServoState_id
{
public:
  Init_PWMServoState_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PWMServoState_position id(::ros_robot_controller_msgs::msg::PWMServoState::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_PWMServoState_position(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::PWMServoState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::PWMServoState>()
{
  return ros_robot_controller_msgs::msg::builder::Init_PWMServoState_id();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__PWM_SERVO_STATE__BUILDER_HPP_
