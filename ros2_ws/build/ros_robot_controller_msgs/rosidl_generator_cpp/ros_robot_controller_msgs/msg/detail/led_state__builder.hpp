// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/LedState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/led_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__LED_STATE__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__LED_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/led_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_LedState_repeat
{
public:
  explicit Init_LedState_repeat(::ros_robot_controller_msgs::msg::LedState & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::LedState repeat(::ros_robot_controller_msgs::msg::LedState::_repeat_type arg)
  {
    msg_.repeat = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::LedState msg_;
};

class Init_LedState_off_time
{
public:
  explicit Init_LedState_off_time(::ros_robot_controller_msgs::msg::LedState & msg)
  : msg_(msg)
  {}
  Init_LedState_repeat off_time(::ros_robot_controller_msgs::msg::LedState::_off_time_type arg)
  {
    msg_.off_time = std::move(arg);
    return Init_LedState_repeat(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::LedState msg_;
};

class Init_LedState_on_time
{
public:
  explicit Init_LedState_on_time(::ros_robot_controller_msgs::msg::LedState & msg)
  : msg_(msg)
  {}
  Init_LedState_off_time on_time(::ros_robot_controller_msgs::msg::LedState::_on_time_type arg)
  {
    msg_.on_time = std::move(arg);
    return Init_LedState_off_time(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::LedState msg_;
};

class Init_LedState_id
{
public:
  Init_LedState_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LedState_on_time id(::ros_robot_controller_msgs::msg::LedState::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_LedState_on_time(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::LedState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::LedState>()
{
  return ros_robot_controller_msgs::msg::builder::Init_LedState_id();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__LED_STATE__BUILDER_HPP_
