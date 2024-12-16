// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/MotorsState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/motors_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTORS_STATE__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTORS_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/motors_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorsState_data
{
public:
  Init_MotorsState_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros_robot_controller_msgs::msg::MotorsState data(::ros_robot_controller_msgs::msg::MotorsState::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::MotorsState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::MotorsState>()
{
  return ros_robot_controller_msgs::msg::builder::Init_MotorsState_data();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__MOTORS_STATE__BUILDER_HPP_
