// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/BuzzerState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/buzzer_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUZZER_STATE__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUZZER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/buzzer_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_BuzzerState_repeat
{
public:
  explicit Init_BuzzerState_repeat(::ros_robot_controller_msgs::msg::BuzzerState & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::BuzzerState repeat(::ros_robot_controller_msgs::msg::BuzzerState::_repeat_type arg)
  {
    msg_.repeat = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BuzzerState msg_;
};

class Init_BuzzerState_off_time
{
public:
  explicit Init_BuzzerState_off_time(::ros_robot_controller_msgs::msg::BuzzerState & msg)
  : msg_(msg)
  {}
  Init_BuzzerState_repeat off_time(::ros_robot_controller_msgs::msg::BuzzerState::_off_time_type arg)
  {
    msg_.off_time = std::move(arg);
    return Init_BuzzerState_repeat(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BuzzerState msg_;
};

class Init_BuzzerState_on_time
{
public:
  explicit Init_BuzzerState_on_time(::ros_robot_controller_msgs::msg::BuzzerState & msg)
  : msg_(msg)
  {}
  Init_BuzzerState_off_time on_time(::ros_robot_controller_msgs::msg::BuzzerState::_on_time_type arg)
  {
    msg_.on_time = std::move(arg);
    return Init_BuzzerState_off_time(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BuzzerState msg_;
};

class Init_BuzzerState_freq
{
public:
  Init_BuzzerState_freq()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BuzzerState_on_time freq(::ros_robot_controller_msgs::msg::BuzzerState::_freq_type arg)
  {
    msg_.freq = std::move(arg);
    return Init_BuzzerState_on_time(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BuzzerState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::BuzzerState>()
{
  return ros_robot_controller_msgs::msg::builder::Init_BuzzerState_freq();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUZZER_STATE__BUILDER_HPP_
