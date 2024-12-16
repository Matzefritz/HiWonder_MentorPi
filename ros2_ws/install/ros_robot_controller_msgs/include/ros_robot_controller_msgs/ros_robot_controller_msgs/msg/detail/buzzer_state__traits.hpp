// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_robot_controller_msgs:msg/BuzzerState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/buzzer_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUZZER_STATE__TRAITS_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUZZER_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_robot_controller_msgs/msg/detail/buzzer_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros_robot_controller_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BuzzerState & msg,
  std::ostream & out)
{
  out << "{";
  // member: freq
  {
    out << "freq: ";
    rosidl_generator_traits::value_to_yaml(msg.freq, out);
    out << ", ";
  }

  // member: on_time
  {
    out << "on_time: ";
    rosidl_generator_traits::value_to_yaml(msg.on_time, out);
    out << ", ";
  }

  // member: off_time
  {
    out << "off_time: ";
    rosidl_generator_traits::value_to_yaml(msg.off_time, out);
    out << ", ";
  }

  // member: repeat
  {
    out << "repeat: ";
    rosidl_generator_traits::value_to_yaml(msg.repeat, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BuzzerState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: freq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "freq: ";
    rosidl_generator_traits::value_to_yaml(msg.freq, out);
    out << "\n";
  }

  // member: on_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "on_time: ";
    rosidl_generator_traits::value_to_yaml(msg.on_time, out);
    out << "\n";
  }

  // member: off_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "off_time: ";
    rosidl_generator_traits::value_to_yaml(msg.off_time, out);
    out << "\n";
  }

  // member: repeat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "repeat: ";
    rosidl_generator_traits::value_to_yaml(msg.repeat, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BuzzerState & msg, bool use_flow_style = false)
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
  const ros_robot_controller_msgs::msg::BuzzerState & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_robot_controller_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_robot_controller_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_robot_controller_msgs::msg::BuzzerState & msg)
{
  return ros_robot_controller_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_robot_controller_msgs::msg::BuzzerState>()
{
  return "ros_robot_controller_msgs::msg::BuzzerState";
}

template<>
inline const char * name<ros_robot_controller_msgs::msg::BuzzerState>()
{
  return "ros_robot_controller_msgs/msg/BuzzerState";
}

template<>
struct has_fixed_size<ros_robot_controller_msgs::msg::BuzzerState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros_robot_controller_msgs::msg::BuzzerState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros_robot_controller_msgs::msg::BuzzerState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUZZER_STATE__TRAITS_HPP_
