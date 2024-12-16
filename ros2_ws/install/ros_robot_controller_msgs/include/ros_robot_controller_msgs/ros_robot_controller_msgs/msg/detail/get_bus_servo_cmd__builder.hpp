// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_robot_controller_msgs:msg/GetBusServoCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/get_bus_servo_cmd.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_BUS_SERVO_CMD__BUILDER_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_BUS_SERVO_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_robot_controller_msgs/msg/detail/get_bus_servo_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_robot_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_GetBusServoCmd_get_torque_state
{
public:
  explicit Init_GetBusServoCmd_get_torque_state(::ros_robot_controller_msgs::msg::GetBusServoCmd & msg)
  : msg_(msg)
  {}
  ::ros_robot_controller_msgs::msg::GetBusServoCmd get_torque_state(::ros_robot_controller_msgs::msg::GetBusServoCmd::_get_torque_state_type arg)
  {
    msg_.get_torque_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::GetBusServoCmd msg_;
};

class Init_GetBusServoCmd_get_max_temperature_limit
{
public:
  explicit Init_GetBusServoCmd_get_max_temperature_limit(::ros_robot_controller_msgs::msg::GetBusServoCmd & msg)
  : msg_(msg)
  {}
  Init_GetBusServoCmd_get_torque_state get_max_temperature_limit(::ros_robot_controller_msgs::msg::GetBusServoCmd::_get_max_temperature_limit_type arg)
  {
    msg_.get_max_temperature_limit = std::move(arg);
    return Init_GetBusServoCmd_get_torque_state(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::GetBusServoCmd msg_;
};

class Init_GetBusServoCmd_get_voltage_limit
{
public:
  explicit Init_GetBusServoCmd_get_voltage_limit(::ros_robot_controller_msgs::msg::GetBusServoCmd & msg)
  : msg_(msg)
  {}
  Init_GetBusServoCmd_get_max_temperature_limit get_voltage_limit(::ros_robot_controller_msgs::msg::GetBusServoCmd::_get_voltage_limit_type arg)
  {
    msg_.get_voltage_limit = std::move(arg);
    return Init_GetBusServoCmd_get_max_temperature_limit(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::GetBusServoCmd msg_;
};

class Init_GetBusServoCmd_get_position_limit
{
public:
  explicit Init_GetBusServoCmd_get_position_limit(::ros_robot_controller_msgs::msg::GetBusServoCmd & msg)
  : msg_(msg)
  {}
  Init_GetBusServoCmd_get_voltage_limit get_position_limit(::ros_robot_controller_msgs::msg::GetBusServoCmd::_get_position_limit_type arg)
  {
    msg_.get_position_limit = std::move(arg);
    return Init_GetBusServoCmd_get_voltage_limit(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::GetBusServoCmd msg_;
};

class Init_GetBusServoCmd_get_temperature
{
public:
  explicit Init_GetBusServoCmd_get_temperature(::ros_robot_controller_msgs::msg::GetBusServoCmd & msg)
  : msg_(msg)
  {}
  Init_GetBusServoCmd_get_position_limit get_temperature(::ros_robot_controller_msgs::msg::GetBusServoCmd::_get_temperature_type arg)
  {
    msg_.get_temperature = std::move(arg);
    return Init_GetBusServoCmd_get_position_limit(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::GetBusServoCmd msg_;
};

class Init_GetBusServoCmd_get_voltage
{
public:
  explicit Init_GetBusServoCmd_get_voltage(::ros_robot_controller_msgs::msg::GetBusServoCmd & msg)
  : msg_(msg)
  {}
  Init_GetBusServoCmd_get_temperature get_voltage(::ros_robot_controller_msgs::msg::GetBusServoCmd::_get_voltage_type arg)
  {
    msg_.get_voltage = std::move(arg);
    return Init_GetBusServoCmd_get_temperature(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::GetBusServoCmd msg_;
};

class Init_GetBusServoCmd_get_offset
{
public:
  explicit Init_GetBusServoCmd_get_offset(::ros_robot_controller_msgs::msg::GetBusServoCmd & msg)
  : msg_(msg)
  {}
  Init_GetBusServoCmd_get_voltage get_offset(::ros_robot_controller_msgs::msg::GetBusServoCmd::_get_offset_type arg)
  {
    msg_.get_offset = std::move(arg);
    return Init_GetBusServoCmd_get_voltage(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::GetBusServoCmd msg_;
};

class Init_GetBusServoCmd_get_position
{
public:
  explicit Init_GetBusServoCmd_get_position(::ros_robot_controller_msgs::msg::GetBusServoCmd & msg)
  : msg_(msg)
  {}
  Init_GetBusServoCmd_get_offset get_position(::ros_robot_controller_msgs::msg::GetBusServoCmd::_get_position_type arg)
  {
    msg_.get_position = std::move(arg);
    return Init_GetBusServoCmd_get_offset(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::GetBusServoCmd msg_;
};

class Init_GetBusServoCmd_get_id
{
public:
  explicit Init_GetBusServoCmd_get_id(::ros_robot_controller_msgs::msg::GetBusServoCmd & msg)
  : msg_(msg)
  {}
  Init_GetBusServoCmd_get_position get_id(::ros_robot_controller_msgs::msg::GetBusServoCmd::_get_id_type arg)
  {
    msg_.get_id = std::move(arg);
    return Init_GetBusServoCmd_get_position(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::GetBusServoCmd msg_;
};

class Init_GetBusServoCmd_id
{
public:
  Init_GetBusServoCmd_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetBusServoCmd_get_id id(::ros_robot_controller_msgs::msg::GetBusServoCmd::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_GetBusServoCmd_get_id(msg_);
  }

private:
  ::ros_robot_controller_msgs::msg::GetBusServoCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_robot_controller_msgs::msg::GetBusServoCmd>()
{
  return ros_robot_controller_msgs::msg::builder::Init_GetBusServoCmd_id();
}

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_BUS_SERVO_CMD__BUILDER_HPP_
