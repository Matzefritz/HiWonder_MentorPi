// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/OLEDState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/oled_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__OLED_STATE__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__OLED_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/oled_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_OLEDState_text
{
public:
  explicit Init_OLEDState_text(::ros_robot_controller_msgs::msg::OLEDState & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::OLEDState text(::ros_robot_controller_msgs::msg::OLEDState::_text_type arg)
  {
    msg_.text = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::OLEDState msg_;
};

class Init_OLEDState_index
{
public:
  Init_OLEDState_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OLEDState_text index(::ros_robot_controller_msgs::msg::OLEDState::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_OLEDState_text(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::OLEDState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::OLEDState>()
{
  return ros_robot_controller_msgs::msg::builder::Init_OLEDState_index();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__OLED_STATE__BUILDER_HPP_
