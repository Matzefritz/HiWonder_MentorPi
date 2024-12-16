// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_robot_controller_msgs:msg/BusServoState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/msg/bus_servo_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUS_SERVO_STATE__TRAITS_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUS_SERVO_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_robot_controller_msgs/msg/detail/bus_servo_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros_robot_controller_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BusServoState & msg,
  std::ostream & out)
{
  out << "{";
  // member: present_id
  {
    if (msg.present_id.size() == 0) {
      out << "present_id: []";
    } else {
      out << "present_id: [";
      size_t pending_items = msg.present_id.size();
      for (auto item : msg.present_id) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: target_id
  {
    if (msg.target_id.size() == 0) {
      out << "target_id: []";
    } else {
      out << "target_id: [";
      size_t pending_items = msg.target_id.size();
      for (auto item : msg.target_id) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: position
  {
    if (msg.position.size() == 0) {
      out << "position: []";
    } else {
      out << "position: [";
      size_t pending_items = msg.position.size();
      for (auto item : msg.position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: offset
  {
    if (msg.offset.size() == 0) {
      out << "offset: []";
    } else {
      out << "offset: [";
      size_t pending_items = msg.offset.size();
      for (auto item : msg.offset) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: voltage
  {
    if (msg.voltage.size() == 0) {
      out << "voltage: []";
    } else {
      out << "voltage: [";
      size_t pending_items = msg.voltage.size();
      for (auto item : msg.voltage) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: temperature
  {
    if (msg.temperature.size() == 0) {
      out << "temperature: []";
    } else {
      out << "temperature: [";
      size_t pending_items = msg.temperature.size();
      for (auto item : msg.temperature) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: position_limit
  {
    if (msg.position_limit.size() == 0) {
      out << "position_limit: []";
    } else {
      out << "position_limit: [";
      size_t pending_items = msg.position_limit.size();
      for (auto item : msg.position_limit) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: voltage_limit
  {
    if (msg.voltage_limit.size() == 0) {
      out << "voltage_limit: []";
    } else {
      out << "voltage_limit: [";
      size_t pending_items = msg.voltage_limit.size();
      for (auto item : msg.voltage_limit) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: max_temperature_limit
  {
    if (msg.max_temperature_limit.size() == 0) {
      out << "max_temperature_limit: []";
    } else {
      out << "max_temperature_limit: [";
      size_t pending_items = msg.max_temperature_limit.size();
      for (auto item : msg.max_temperature_limit) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: enable_torque
  {
    if (msg.enable_torque.size() == 0) {
      out << "enable_torque: []";
    } else {
      out << "enable_torque: [";
      size_t pending_items = msg.enable_torque.size();
      for (auto item : msg.enable_torque) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: save_offset
  {
    if (msg.save_offset.size() == 0) {
      out << "save_offset: []";
    } else {
      out << "save_offset: [";
      size_t pending_items = msg.save_offset.size();
      for (auto item : msg.save_offset) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: stop
  {
    if (msg.stop.size() == 0) {
      out << "stop: []";
    } else {
      out << "stop: [";
      size_t pending_items = msg.stop.size();
      for (auto item : msg.stop) {
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
  const BusServoState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: present_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.present_id.size() == 0) {
      out << "present_id: []\n";
    } else {
      out << "present_id:\n";
      for (auto item : msg.present_id) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: target_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.target_id.size() == 0) {
      out << "target_id: []\n";
    } else {
      out << "target_id:\n";
      for (auto item : msg.target_id) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.position.size() == 0) {
      out << "position: []\n";
    } else {
      out << "position:\n";
      for (auto item : msg.position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.offset.size() == 0) {
      out << "offset: []\n";
    } else {
      out << "offset:\n";
      for (auto item : msg.offset) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.voltage.size() == 0) {
      out << "voltage: []\n";
    } else {
      out << "voltage:\n";
      for (auto item : msg.voltage) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.temperature.size() == 0) {
      out << "temperature: []\n";
    } else {
      out << "temperature:\n";
      for (auto item : msg.temperature) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: position_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.position_limit.size() == 0) {
      out << "position_limit: []\n";
    } else {
      out << "position_limit:\n";
      for (auto item : msg.position_limit) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: voltage_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.voltage_limit.size() == 0) {
      out << "voltage_limit: []\n";
    } else {
      out << "voltage_limit:\n";
      for (auto item : msg.voltage_limit) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: max_temperature_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.max_temperature_limit.size() == 0) {
      out << "max_temperature_limit: []\n";
    } else {
      out << "max_temperature_limit:\n";
      for (auto item : msg.max_temperature_limit) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: enable_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.enable_torque.size() == 0) {
      out << "enable_torque: []\n";
    } else {
      out << "enable_torque:\n";
      for (auto item : msg.enable_torque) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: save_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.save_offset.size() == 0) {
      out << "save_offset: []\n";
    } else {
      out << "save_offset:\n";
      for (auto item : msg.save_offset) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.stop.size() == 0) {
      out << "stop: []\n";
    } else {
      out << "stop:\n";
      for (auto item : msg.stop) {
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

inline std::string to_yaml(const BusServoState & msg, bool use_flow_style = false)
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
  const ros_robot_controller_msgs::msg::BusServoState & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_robot_controller_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_robot_controller_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_robot_controller_msgs::msg::BusServoState & msg)
{
  return ros_robot_controller_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_robot_controller_msgs::msg::BusServoState>()
{
  return "ros_robot_controller_msgs::msg::BusServoState";
}

template<>
inline const char * name<ros_robot_controller_msgs::msg::BusServoState>()
{
  return "ros_robot_controller_msgs/msg/BusServoState";
}

template<>
struct has_fixed_size<ros_robot_controller_msgs::msg::BusServoState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_robot_controller_msgs::msg::BusServoState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros_robot_controller_msgs::msg::BusServoState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_ROBOT_CONTROLLER_MSGS__MSG__DETAIL__BUS_SERVO_STATE__TRAITS_HPP_
