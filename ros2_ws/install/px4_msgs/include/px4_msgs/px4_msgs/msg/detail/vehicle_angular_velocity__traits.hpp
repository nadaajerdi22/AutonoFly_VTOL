// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/VehicleAngularVelocity.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "px4_msgs/msg/vehicle_angular_velocity.hpp"


#ifndef PX4_MSGS__MSG__DETAIL__VEHICLE_ANGULAR_VELOCITY__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__VEHICLE_ANGULAR_VELOCITY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "px4_msgs/msg/detail/vehicle_angular_velocity__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace px4_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const VehicleAngularVelocity & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: timestamp_sample
  {
    out << "timestamp_sample: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp_sample, out);
    out << ", ";
  }

  // member: xyz
  {
    if (msg.xyz.size() == 0) {
      out << "xyz: []";
    } else {
      out << "xyz: [";
      size_t pending_items = msg.xyz.size();
      for (auto item : msg.xyz) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: xyz_derivative
  {
    if (msg.xyz_derivative.size() == 0) {
      out << "xyz_derivative: []";
    } else {
      out << "xyz_derivative: [";
      size_t pending_items = msg.xyz_derivative.size();
      for (auto item : msg.xyz_derivative) {
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
  const VehicleAngularVelocity & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: timestamp_sample
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp_sample: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp_sample, out);
    out << "\n";
  }

  // member: xyz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.xyz.size() == 0) {
      out << "xyz: []\n";
    } else {
      out << "xyz:\n";
      for (auto item : msg.xyz) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: xyz_derivative
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.xyz_derivative.size() == 0) {
      out << "xyz_derivative: []\n";
    } else {
      out << "xyz_derivative:\n";
      for (auto item : msg.xyz_derivative) {
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

inline std::string to_yaml(const VehicleAngularVelocity & msg, bool use_flow_style = false)
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

}  // namespace px4_msgs

namespace rosidl_generator_traits
{

[[deprecated("use px4_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const px4_msgs::msg::VehicleAngularVelocity & msg,
  std::ostream & out, size_t indentation = 0)
{
  px4_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use px4_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const px4_msgs::msg::VehicleAngularVelocity & msg)
{
  return px4_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<px4_msgs::msg::VehicleAngularVelocity>()
{
  return "px4_msgs::msg::VehicleAngularVelocity";
}

template<>
inline const char * name<px4_msgs::msg::VehicleAngularVelocity>()
{
  return "px4_msgs/msg/VehicleAngularVelocity";
}

template<>
struct has_fixed_size<px4_msgs::msg::VehicleAngularVelocity>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::VehicleAngularVelocity>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::VehicleAngularVelocity>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__VEHICLE_ANGULAR_VELOCITY__TRAITS_HPP_
