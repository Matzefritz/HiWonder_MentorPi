// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_robot_controller_msgs:msg/Sbus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/sbus.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SBUS__TRAITS_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SBUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_robot_controller_msgs/msg/detail/sbus__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ros_robot_controller_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Sbus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: channel
  {
    if (msg.channel.size() == 0) {
      out << "channel: []";
    } else {
      out << "channel: [";
      size_t pending_items = msg.channel.size();
      for (auto item : msg.channel) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Sbus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: channel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.channel.size() == 0) {
      out << "channel: []\n";
    } else {
      out << "channel:\n";
      for (auto item : msg.channel) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Sbus & msg, bool use_flow_style = false)
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
  const ros_robot_controller_msgs::msg::Sbus & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_robot_controller_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_robot_controller_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_robot_controller_msgs::msg::Sbus & msg)
{
  return ros_robot_controller_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_robot_controller_msgs::msg::Sbus>()
{
  return "ros_robot_controller_msgs::msg::Sbus";
}

template<>
inline const char * name<ros_robot_controller_msgs::msg::Sbus>()
{
  return "ros_robot_controller_msgs/msg/Sbus";
}

template<>
struct has_fixed_size<ros_robot_controller_msgs::msg::Sbus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_robot_controller_msgs::msg::Sbus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros_robot_controller_msgs::msg::Sbus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__SBUS__TRAITS_HPP_
