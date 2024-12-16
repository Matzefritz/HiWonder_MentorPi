// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/SetBusServoState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/set_bus_servo_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SET_BUS_SERVO_STATE__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SET_BUS_SERVO_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/set_bus_servo_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_SetBusServoState_duration
{
public:
  explicit Init_SetBusServoState_duration(::ros_robot_controller_msgs::msg::SetBusServoState & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::SetBusServoState duration(::ros_robot_controller_msgs::msg::SetBusServoState::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::SetBusServoState msg_;
};

class Init_SetBusServoState_state
{
public:
  Init_SetBusServoState_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetBusServoState_duration state(::ros_robot_controller_msgs::msg::SetBusServoState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_SetBusServoState_duration(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::SetBusServoState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::SetBusServoState>()
{
  return ros_robot_controller_msgs::msg::builder::Init_SetBusServoState_state();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SET_BUS_SERVO_STATE__BUILDER_HPP_
