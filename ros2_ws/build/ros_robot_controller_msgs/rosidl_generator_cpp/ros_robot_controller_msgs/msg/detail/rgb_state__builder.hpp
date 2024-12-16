// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/RGBState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/rgb_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__RGB_STATE__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__RGB_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/rgb_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_RGBState_blue
{
public:
  explicit Init_RGBState_blue(::ros_robot_controller_msgs::msg::RGBState & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::RGBState blue(::ros_robot_controller_msgs::msg::RGBState::_blue_type arg)
  {
    msg_.blue = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::RGBState msg_;
};

class Init_RGBState_green
{
public:
  explicit Init_RGBState_green(::ros_robot_controller_msgs::msg::RGBState & msg)
  : msg_(msg)
  {}
  Init_RGBState_blue green(::ros_robot_controller_msgs::msg::RGBState::_green_type arg)
  {
    msg_.green = std::move(arg);
    return Init_RGBState_blue(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::RGBState msg_;
};

class Init_RGBState_red
{
public:
  explicit Init_RGBState_red(::ros_robot_controller_msgs::msg::RGBState & msg)
  : msg_(msg)
  {}
  Init_RGBState_green red(::ros_robot_controller_msgs::msg::RGBState::_red_type arg)
  {
    msg_.red = std::move(arg);
    return Init_RGBState_green(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::RGBState msg_;
};

class Init_RGBState_index
{
public:
  Init_RGBState_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RGBState_red index(::ros_robot_controller_msgs::msg::RGBState::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_RGBState_red(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::RGBState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::RGBState>()
{
  return ros_robot_controller_msgs::msg::builder::Init_RGBState_index();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__RGB_STATE__BUILDER_HPP_
