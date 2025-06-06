// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:srv/VehicleCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "px4_msgs/srv/vehicle_command.hpp"


#ifndef PX4_MSGS__SRV__DETAIL__VEHICLE_COMMAND__TRAITS_HPP_
#define PX4_MSGS__SRV__DETAIL__VEHICLE_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "px4_msgs/srv/detail/vehicle_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'request'
#include "px4_msgs/msg/detail/vehicle_command__traits.hpp"

namespace px4_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const VehicleCommand_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: request
  {
    out << "request: ";
    to_flow_style_yaml(msg.request, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VehicleCommand_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request:\n";
    to_block_style_yaml(msg.request, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VehicleCommand_Request & msg, bool use_flow_style = false)
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

}  // namespace px4_msgs

namespace rosidl_generator_traits
{

[[deprecated("use px4_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const px4_msgs::srv::VehicleCommand_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  px4_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use px4_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const px4_msgs::srv::VehicleCommand_Request & msg)
{
  return px4_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<px4_msgs::srv::VehicleCommand_Request>()
{
  return "px4_msgs::srv::VehicleCommand_Request";
}

template<>
inline const char * name<px4_msgs::srv::VehicleCommand_Request>()
{
  return "px4_msgs/srv/VehicleCommand_Request";
}

template<>
struct has_fixed_size<px4_msgs::srv::VehicleCommand_Request>
  : std::integral_constant<bool, has_fixed_size<px4_msgs::msg::VehicleCommand>::value> {};

template<>
struct has_bounded_size<px4_msgs::srv::VehicleCommand_Request>
  : std::integral_constant<bool, has_bounded_size<px4_msgs::msg::VehicleCommand>::value> {};

template<>
struct is_message<px4_msgs::srv::VehicleCommand_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'reply'
#include "px4_msgs/msg/detail/vehicle_command_ack__traits.hpp"

namespace px4_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const VehicleCommand_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: reply
  {
    out << "reply: ";
    to_flow_style_yaml(msg.reply, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VehicleCommand_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: reply
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reply:\n";
    to_block_style_yaml(msg.reply, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VehicleCommand_Response & msg, bool use_flow_style = false)
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

}  // namespace px4_msgs

namespace rosidl_generator_traits
{

[[deprecated("use px4_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const px4_msgs::srv::VehicleCommand_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  px4_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use px4_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const px4_msgs::srv::VehicleCommand_Response & msg)
{
  return px4_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<px4_msgs::srv::VehicleCommand_Response>()
{
  return "px4_msgs::srv::VehicleCommand_Response";
}

template<>
inline const char * name<px4_msgs::srv::VehicleCommand_Response>()
{
  return "px4_msgs/srv/VehicleCommand_Response";
}

template<>
struct has_fixed_size<px4_msgs::srv::VehicleCommand_Response>
  : std::integral_constant<bool, has_fixed_size<px4_msgs::msg::VehicleCommandAck>::value> {};

template<>
struct has_bounded_size<px4_msgs::srv::VehicleCommand_Response>
  : std::integral_constant<bool, has_bounded_size<px4_msgs::msg::VehicleCommandAck>::value> {};

template<>
struct is_message<px4_msgs::srv::VehicleCommand_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace px4_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const VehicleCommand_Event & msg,
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
  const VehicleCommand_Event & msg,
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

inline std::string to_yaml(const VehicleCommand_Event & msg, bool use_flow_style = false)
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

}  // namespace px4_msgs

namespace rosidl_generator_traits
{

[[deprecated("use px4_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const px4_msgs::srv::VehicleCommand_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  px4_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use px4_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const px4_msgs::srv::VehicleCommand_Event & msg)
{
  return px4_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<px4_msgs::srv::VehicleCommand_Event>()
{
  return "px4_msgs::srv::VehicleCommand_Event";
}

template<>
inline const char * name<px4_msgs::srv::VehicleCommand_Event>()
{
  return "px4_msgs/srv/VehicleCommand_Event";
}

template<>
struct has_fixed_size<px4_msgs::srv::VehicleCommand_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<px4_msgs::srv::VehicleCommand_Event>
  : std::integral_constant<bool, has_bounded_size<px4_msgs::srv::VehicleCommand_Request>::value && has_bounded_size<px4_msgs::srv::VehicleCommand_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<px4_msgs::srv::VehicleCommand_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<px4_msgs::srv::VehicleCommand>()
{
  return "px4_msgs::srv::VehicleCommand";
}

template<>
inline const char * name<px4_msgs::srv::VehicleCommand>()
{
  return "px4_msgs/srv/VehicleCommand";
}

template<>
struct has_fixed_size<px4_msgs::srv::VehicleCommand>
  : std::integral_constant<
    bool,
    has_fixed_size<px4_msgs::srv::VehicleCommand_Request>::value &&
    has_fixed_size<px4_msgs::srv::VehicleCommand_Response>::value
  >
{
};

template<>
struct has_bounded_size<px4_msgs::srv::VehicleCommand>
  : std::integral_constant<
    bool,
    has_bounded_size<px4_msgs::srv::VehicleCommand_Request>::value &&
    has_bounded_size<px4_msgs::srv::VehicleCommand_Response>::value
  >
{
};

template<>
struct is_service<px4_msgs::srv::VehicleCommand>
  : std::true_type
{
};

template<>
struct is_service_request<px4_msgs::srv::VehicleCommand_Request>
  : std::true_type
{
};

template<>
struct is_service_response<px4_msgs::srv::VehicleCommand_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__SRV__DETAIL__VEHICLE_COMMAND__TRAITS_HPP_
