// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/ButtonState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/button_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUTTON_STATE__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUTTON_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/button_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_ButtonState_state
{
public:
  explicit Init_ButtonState_state(::ros_robot_controller_msgs::msg::ButtonState & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::ButtonState state(::ros_robot_controller_msgs::msg::ButtonState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::ButtonState msg_;
};

class Init_ButtonState_id
{
public:
  Init_ButtonState_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ButtonState_state id(::ros_robot_controller_msgs::msg::ButtonState::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_ButtonState_state(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::ButtonState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::ButtonState>()
{
  return ros_robot_controller_msgs::msg::builder::Init_ButtonState_id();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUTTON_STATE__BUILDER_HPP_
