// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/BusServoState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/bus_servo_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUS_SERVO_STATE__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUS_SERVO_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/bus_servo_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_BusServoState_stop
{
public:
  explicit Init_BusServoState_stop(::ros_robot_controller_msgs::msg::BusServoState & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::BusServoState stop(::ros_robot_controller_msgs::msg::BusServoState::_stop_type arg)
  {
    msg_.stop = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BusServoState msg_;
};

class Init_BusServoState_save_offset
{
public:
  explicit Init_BusServoState_save_offset(::ros_robot_controller_msgs::msg::BusServoState & msg)
  : msg_(msg)
  {}
  Init_BusServoState_stop save_offset(::ros_robot_controller_msgs::msg::BusServoState::_save_offset_type arg)
  {
    msg_.save_offset = std::move(arg);
    return Init_BusServoState_stop(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BusServoState msg_;
};

class Init_BusServoState_enable_torque
{
public:
  explicit Init_BusServoState_enable_torque(::ros_robot_controller_msgs::msg::BusServoState & msg)
  : msg_(msg)
  {}
  Init_BusServoState_save_offset enable_torque(::ros_robot_controller_msgs::msg::BusServoState::_enable_torque_type arg)
  {
    msg_.enable_torque = std::move(arg);
    return Init_BusServoState_save_offset(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BusServoState msg_;
};

class Init_BusServoState_max_temperature_limit
{
public:
  explicit Init_BusServoState_max_temperature_limit(::ros_robot_controller_msgs::msg::BusServoState & msg)
  : msg_(msg)
  {}
  Init_BusServoState_enable_torque max_temperature_limit(::ros_robot_controller_msgs::msg::BusServoState::_max_temperature_limit_type arg)
  {
    msg_.max_temperature_limit = std::move(arg);
    return Init_BusServoState_enable_torque(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BusServoState msg_;
};

class Init_BusServoState_voltage_limit
{
public:
  explicit Init_BusServoState_voltage_limit(::ros_robot_controller_msgs::msg::BusServoState & msg)
  : msg_(msg)
  {}
  Init_BusServoState_max_temperature_limit voltage_limit(::ros_robot_controller_msgs::msg::BusServoState::_voltage_limit_type arg)
  {
    msg_.voltage_limit = std::move(arg);
    return Init_BusServoState_max_temperature_limit(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BusServoState msg_;
};

class Init_BusServoState_position_limit
{
public:
  explicit Init_BusServoState_position_limit(::ros_robot_controller_msgs::msg::BusServoState & msg)
  : msg_(msg)
  {}
  Init_BusServoState_voltage_limit position_limit(::ros_robot_controller_msgs::msg::BusServoState::_position_limit_type arg)
  {
    msg_.position_limit = std::move(arg);
    return Init_BusServoState_voltage_limit(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BusServoState msg_;
};

class Init_BusServoState_temperature
{
public:
  explicit Init_BusServoState_temperature(::ros_robot_controller_msgs::msg::BusServoState & msg)
  : msg_(msg)
  {}
  Init_BusServoState_position_limit temperature(::ros_robot_controller_msgs::msg::BusServoState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_BusServoState_position_limit(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BusServoState msg_;
};

class Init_BusServoState_voltage
{
public:
  explicit Init_BusServoState_voltage(::ros_robot_controller_msgs::msg::BusServoState & msg)
  : msg_(msg)
  {}
  Init_BusServoState_temperature voltage(::ros_robot_controller_msgs::msg::BusServoState::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_BusServoState_temperature(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BusServoState msg_;
};

class Init_BusServoState_offset
{
public:
  explicit Init_BusServoState_offset(::ros_robot_controller_msgs::msg::BusServoState & msg)
  : msg_(msg)
  {}
  Init_BusServoState_voltage offset(::ros_robot_controller_msgs::msg::BusServoState::_offset_type arg)
  {
    msg_.offset = std::move(arg);
    return Init_BusServoState_voltage(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BusServoState msg_;
};

class Init_BusServoState_position
{
public:
  explicit Init_BusServoState_position(::ros_robot_controller_msgs::msg::BusServoState & msg)
  : msg_(msg)
  {}
  Init_BusServoState_offset position(::ros_robot_controller_msgs::msg::BusServoState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_BusServoState_offset(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BusServoState msg_;
};

class Init_BusServoState_target_id
{
public:
  explicit Init_BusServoState_target_id(::ros_robot_controller_msgs::msg::BusServoState & msg)
  : msg_(msg)
  {}
  Init_BusServoState_position target_id(::ros_robot_controller_msgs::msg::BusServoState::_target_id_type arg)
  {
    msg_.target_id = std::move(arg);
    return Init_BusServoState_position(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BusServoState msg_;
};

class Init_BusServoState_present_id
{
public:
  Init_BusServoState_present_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BusServoState_target_id present_id(::ros_robot_controller_msgs::msg::BusServoState::_present_id_type arg)
  {
    msg_.present_id = std::move(arg);
    return Init_BusServoState_target_id(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::BusServoState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::BusServoState>()
{
  return ros_robot_controller_msgs::msg::builder::Init_BusServoState_present_id();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUS_SERVO_STATE__BUILDER_HPP_
