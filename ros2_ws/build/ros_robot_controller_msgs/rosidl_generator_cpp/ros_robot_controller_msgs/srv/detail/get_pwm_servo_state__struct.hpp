// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_robot_controller_msgs:srv/GetPWMServoState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros_robot_controller_msgs/srv/get_pwm_servo_state.hpp"


#ifndef ROS_ROBOT_CONTROLLER_MSGS__SRV__DETAIL__GET_PWM_SERVO_STATE__STRUCT_HPP_
#define ROS_ROBOT_CONTROLLER_MSGS__SRV__DETAIL__GET_PWM_SERVO_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'cmd'
#include "ros_robot_controller_msgs/msg/detail/get_pwm_servo_cmd__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_robot_controller_msgs__srv__GetPWMServoState_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros_robot_controller_msgs__srv__GetPWMServoState_Request __declspec(deprecated)
#endif

namespace ros_robot_controller_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPWMServoState_Request_
{
  using Type = GetPWMServoState_Request_<ContainerAllocator>;

  explicit GetPWMServoState_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit GetPWMServoState_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _cmd_type =
    std::vector<ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator>>>;
  _cmd_type cmd;

  // setters for named parameter idiom
  Type & set__cmd(
    const std::vector<ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros_robot_controller_msgs::msg::GetPWMServoCmd_<ContainerAllocator>>> & _arg)
  {
    this->cmd = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_robot_controller_msgs__srv__GetPWMServoState_Request
    std::shared_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_robot_controller_msgs__srv__GetPWMServoState_Request
    std::shared_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPWMServoState_Request_ & other) const
  {
    if (this->cmd != other.cmd) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPWMServoState_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPWMServoState_Request_

// alias to use template instance with default allocator
using GetPWMServoState_Request =
  ros_robot_controller_msgs::srv::GetPWMServoState_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros_robot_controller_msgs


// Include directives for member types
// Member 'state'
#include "ros_robot_controller_msgs/msg/detail/pwm_servo_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_robot_controller_msgs__srv__GetPWMServoState_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros_robot_controller_msgs__srv__GetPWMServoState_Response __declspec(deprecated)
#endif

namespace ros_robot_controller_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPWMServoState_Response_
{
  using Type = GetPWMServoState_Response_<ContainerAllocator>;

  explicit GetPWMServoState_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit GetPWMServoState_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _state_type =
    std::vector<ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator>>>;
  _state_type state;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__state(
    const std::vector<ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros_robot_controller_msgs::msg::PWMServoState_<ContainerAllocator>>> & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_robot_controller_msgs__srv__GetPWMServoState_Response
    std::shared_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_robot_controller_msgs__srv__GetPWMServoState_Response
    std::shared_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPWMServoState_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPWMServoState_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPWMServoState_Response_

// alias to use template instance with default allocator
using GetPWMServoState_Response =
  ros_robot_controller_msgs::srv::GetPWMServoState_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros_robot_controller_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_robot_controller_msgs__srv__GetPWMServoState_Event __attribute__((deprecated))
#else
# define DEPRECATED__ros_robot_controller_msgs__srv__GetPWMServoState_Event __declspec(deprecated)
#endif

namespace ros_robot_controller_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPWMServoState_Event_
{
  using Type = GetPWMServoState_Event_<ContainerAllocator>;

  explicit GetPWMServoState_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetPWMServoState_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros_robot_controller_msgs::srv::GetPWMServoState_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros_robot_controller_msgs::srv::GetPWMServoState_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_robot_controller_msgs::srv::GetPWMServoState_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_robot_controller_msgs::srv::GetPWMServoState_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_robot_controller_msgs::srv::GetPWMServoState_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_robot_controller_msgs::srv::GetPWMServoState_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_robot_controller_msgs__srv__GetPWMServoState_Event
    std::shared_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_robot_controller_msgs__srv__GetPWMServoState_Event
    std::shared_ptr<ros_robot_controller_msgs::srv::GetPWMServoState_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPWMServoState_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPWMServoState_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPWMServoState_Event_

// alias to use template instance with default allocator
using GetPWMServoState_Event =
  ros_robot_controller_msgs::srv::GetPWMServoState_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros_robot_controller_msgs

namespace ros_robot_controller_msgs
{

namespace srv
{

struct GetPWMServoState
{
  using Request = ros_robot_controller_msgs::srv::GetPWMServoState_Request;
  using Response = ros_robot_controller_msgs::srv::GetPWMServoState_Response;
  using Event = ros_robot_controller_msgs::srv::GetPWMServoState_Event;
};

}  // namespace srv

}  // namespace ros_robot_controller_msgs

#endif  // ROS_ROBOT_CONTROLLER_MSGS__SRV__DETAIL__GET_PWM_SERVO_STATE__STRUCT_HPP_
