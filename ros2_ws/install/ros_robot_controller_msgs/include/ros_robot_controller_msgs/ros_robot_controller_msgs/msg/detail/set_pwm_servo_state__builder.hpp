// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/SetPWMServoState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/set_pwm_servo_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SET_PWM_SERVO_STATE__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SET_PWM_SERVO_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/set_pwm_servo_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_SetPWMServoState_duration
{
public:
  explicit Init_SetPWMServoState_duration(::ros_robot_controller_msgs::msg::SetPWMServoState & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::SetPWMServoState duration(::ros_robot_controller_msgs::msg::SetPWMServoState::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::SetPWMServoState msg_;
};

class Init_SetPWMServoState_state
{
public:
  Init_SetPWMServoState_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPWMServoState_duration state(::ros_robot_controller_msgs::msg::SetPWMServoState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_SetPWMServoState_duration(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::SetPWMServoState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::SetPWMServoState>()
{
  return ros_robot_controller_msgs::msg::builder::Init_SetPWMServoState_state();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SET_PWM_SERVO_STATE__BUILDER_HPP_
