// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_robot_controller_msgs:msg/GetBusServoCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/get_bus_servo_cmd.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_BUS_SERVO_CMD__TRAITS_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_BUS_SERVO_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_robot_controller_msgs/msg/detail/get_bus_servo_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros_robot_controller_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GetBusServoCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: get_id
  {
    out << "get_id: ";
    rosidl_generator_traits::value_to_yaml(msg.get_id, out);
    out << ", ";
  }

  // member: get_position
  {
    out << "get_position: ";
    rosidl_generator_traits::value_to_yaml(msg.get_position, out);
    out << ", ";
  }

  // member: get_offset
  {
    out << "get_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.get_offset, out);
    out << ", ";
  }

  // member: get_voltage
  {
    out << "get_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.get_voltage, out);
    out << ", ";
  }

  // member: get_temperature
  {
    out << "get_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.get_temperature, out);
    out << ", ";
  }

  // member: get_position_limit
  {
    out << "get_position_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.get_position_limit, out);
    out << ", ";
  }

  // member: get_voltage_limit
  {
    out << "get_voltage_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.get_voltage_limit, out);
    out << ", ";
  }

  // member: get_max_temperature_limit
  {
    out << "get_max_temperature_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.get_max_temperature_limit, out);
    out << ", ";
  }

  // member: get_torque_state
  {
    out << "get_torque_state: ";
    rosidl_generator_traits::value_to_yaml(msg.get_torque_state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetBusServoCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: get_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "get_id: ";
    rosidl_generator_traits::value_to_yaml(msg.get_id, out);
    out << "\n";
  }

  // member: get_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "get_position: ";
    rosidl_generator_traits::value_to_yaml(msg.get_position, out);
    out << "\n";
  }

  // member: get_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "get_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.get_offset, out);
    out << "\n";
  }

  // member: get_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "get_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.get_voltage, out);
    out << "\n";
  }

  // member: get_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "get_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.get_temperature, out);
    out << "\n";
  }

  // member: get_position_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "get_position_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.get_position_limit, out);
    out << "\n";
  }

  // member: get_voltage_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "get_voltage_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.get_voltage_limit, out);
    out << "\n";
  }

  // member: get_max_temperature_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "get_max_temperature_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.get_max_temperature_limit, out);
    out << "\n";
  }

  // member: get_torque_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "get_torque_state: ";
    rosidl_generator_traits::value_to_yaml(msg.get_torque_state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetBusServoCmd & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ros_robot_controller_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ros_robot_controller_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros_robot_controller_msgs::msg::GetBusServoCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_robot_controller_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_robot_controller_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_robot_controller_msgs::msg::GetBusServoCmd & msg)
{
  return ros_robot_controller_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_robot_controller_msgs::msg::GetBusServoCmd>()
{
  return "ros_robot_controller_msgs::msg::GetBusServoCmd";
}

template<>
inline const char * name<ros_robot_controller_msgs::msg::GetBusServoCmd>()
{
  return "ros_robot_controller_msgs/msg/GetBusServoCmd";
}

template<>
struct has_fixed_size<ros_robot_controller_msgs::msg::GetBusServoCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros_robot_controller_msgs::msg::GetBusServoCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros_robot_controller_msgs::msg::GetBusServoCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__GET_BUS_SERVO_CMD__TRAITS_HPP_
