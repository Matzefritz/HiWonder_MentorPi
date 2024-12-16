// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_robot_controller_msgs:srv/GetBusServoState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/srv/get_bus_servo_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__SRV__DETAIL__GET_BUS_SERVO_STATE__TRAITS_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__SRV__DETAIL__GET_BUS_SERVO_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_robot_controller_msgs/srv/detail/get_bus_servo_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'cmd'
#include "ros_robot_controller_msgs/msg/detail/get_bus_servo_cmd__traits.hpp"

namespace ros_robot_controller_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetBusServoState_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: cmd
  {
    if (msg.cmd.size() == 0) {
      out << "cmd: []";
    } else {
      out << "cmd: [";
      size_t pending_items = msg.cmd.size();
      for (auto item : msg.cmd) {
        to_flow_style_yaml(item, out);
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
  const GetBusServoState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cmd.size() == 0) {
      out << "cmd: []\n";
    } else {
      out << "cmd:\n";
      for (auto item : msg.cmd) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetBusServoState_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros_robot_controller_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ros_robot_controller_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros_robot_controller_msgs::srv::GetBusServoState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_robot_controller_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_robot_controller_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros_robot_controller_msgs::srv::GetBusServoState_Request & msg)
{
  return ros_robot_controller_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros_robot_controller_msgs::srv::GetBusServoState_Request>()
{
  return "ros_robot_controller_msgs::srv::GetBusServoState_Request";
}

template<>
inline const char * name<ros_robot_controller_msgs::srv::GetBusServoState_Request>()
{
  return "ros_robot_controller_msgs/srv/GetBusServoState_Request";
}

template<>
struct has_fixed_size<ros_robot_controller_msgs::srv::GetBusServoState_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_robot_controller_msgs::srv::GetBusServoState_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros_robot_controller_msgs::srv::GetBusServoState_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'state'
#include "ros_robot_controller_msgs/msg/detail/bus_servo_state__traits.hpp"

namespace ros_robot_controller_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetBusServoState_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: state
  {
    if (msg.state.size() == 0) {
      out << "state: []";
    } else {
      out << "state: [";
      size_t pending_items = msg.state.size();
      for (auto item : msg.state) {
        to_flow_style_yaml(item, out);
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
  const GetBusServoState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.state.size() == 0) {
      out << "state: []\n";
    } else {
      out << "state:\n";
      for (auto item : msg.state) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetBusServoState_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros_robot_controller_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ros_robot_controller_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros_robot_controller_msgs::srv::GetBusServoState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_robot_controller_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_robot_controller_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros_robot_controller_msgs::srv::GetBusServoState_Response & msg)
{
  return ros_robot_controller_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros_robot_controller_msgs::srv::GetBusServoState_Response>()
{
  return "ros_robot_controller_msgs::srv::GetBusServoState_Response";
}

template<>
inline const char * name<ros_robot_controller_msgs::srv::GetBusServoState_Response>()
{
  return "ros_robot_controller_msgs/srv/GetBusServoState_Response";
}

template<>
struct has_fixed_size<ros_robot_controller_msgs::srv::GetBusServoState_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_robot_controller_msgs::srv::GetBusServoState_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros_robot_controller_msgs::srv::GetBusServoState_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace ros_robot_controller_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetBusServoState_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
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
  const GetBusServoState_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetBusServoState_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros_robot_controller_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ros_robot_controller_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros_robot_controller_msgs::srv::GetBusServoState_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_robot_controller_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_robot_controller_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros_robot_controller_msgs::srv::GetBusServoState_Event & msg)
{
  return ros_robot_controller_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros_robot_controller_msgs::srv::GetBusServoState_Event>()
{
  return "ros_robot_controller_msgs::srv::GetBusServoState_Event";
}

template<>
inline const char * name<ros_robot_controller_msgs::srv::GetBusServoState_Event>()
{
  return "ros_robot_controller_msgs/srv/GetBusServoState_Event";
}

template<>
struct has_fixed_size<ros_robot_controller_msgs::srv::GetBusServoState_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_robot_controller_msgs::srv::GetBusServoState_Event>
  : std::integral_constant<bool, has_bounded_size<ros_robot_controller_msgs::srv::GetBusServoState_Request>::value && has_bounded_size<ros_robot_controller_msgs::srv::GetBusServoState_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<ros_robot_controller_msgs::srv::GetBusServoState_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros_robot_controller_msgs::srv::GetBusServoState>()
{
  return "ros_robot_controller_msgs::srv::GetBusServoState";
}

template<>
inline const char * name<ros_robot_controller_msgs::srv::GetBusServoState>()
{
  return "ros_robot_controller_msgs/srv/GetBusServoState";
}

template<>
struct has_fixed_size<ros_robot_controller_msgs::srv::GetBusServoState>
  : std::integral_constant<
    bool,
    has_fixed_size<ros_robot_controller_msgs::srv::GetBusServoState_Request>::value &&
    has_fixed_size<ros_robot_controller_msgs::srv::GetBusServoState_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros_robot_controller_msgs::srv::GetBusServoState>
  : std::integral_constant<
    bool,
    has_bounded_size<ros_robot_controller_msgs::srv::GetBusServoState_Request>::value &&
    has_bounded_size<ros_robot_controller_msgs::srv::GetBusServoState_Response>::value
  >
{
};

template<>
struct is_service<ros_robot_controller_msgs::srv::GetBusServoState>
  : std::true_type
{
};

template<>
struct is_service_request<ros_robot_controller_msgs::srv::GetBusServoState_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros_robot_controller_msgs::srv::GetBusServoState_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS_ROBOT_CONTROLLER_MSGS__SRV__DETAIL__GET_BUS_SERVO_STATE__TRAITS_HPP_
