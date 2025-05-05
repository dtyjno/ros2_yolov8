// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_yolo_msgs:msg/DetectedBox.idl
// generated code does not contain a copyright notice

#ifndef ROS2_YOLO_MSGS__MSG__DETAIL__DETECTED_BOX__TRAITS_HPP_
#define ROS2_YOLO_MSGS__MSG__DETAIL__DETECTED_BOX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_yolo_msgs/msg/detail/detected_box__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_yolo_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DetectedBox & msg,
  std::ostream & out)
{
  out << "{";
  // member: x1
  {
    out << "x1: ";
    rosidl_generator_traits::value_to_yaml(msg.x1, out);
    out << ", ";
  }

  // member: y1
  {
    out << "y1: ";
    rosidl_generator_traits::value_to_yaml(msg.y1, out);
    out << ", ";
  }

  // member: x2
  {
    out << "x2: ";
    rosidl_generator_traits::value_to_yaml(msg.x2, out);
    out << ", ";
  }

  // member: y2
  {
    out << "y2: ";
    rosidl_generator_traits::value_to_yaml(msg.y2, out);
    out << ", ";
  }

  // member: servo
  {
    out << "servo: ";
    rosidl_generator_traits::value_to_yaml(msg.servo, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DetectedBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x1: ";
    rosidl_generator_traits::value_to_yaml(msg.x1, out);
    out << "\n";
  }

  // member: y1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y1: ";
    rosidl_generator_traits::value_to_yaml(msg.y1, out);
    out << "\n";
  }

  // member: x2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x2: ";
    rosidl_generator_traits::value_to_yaml(msg.x2, out);
    out << "\n";
  }

  // member: y2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y2: ";
    rosidl_generator_traits::value_to_yaml(msg.y2, out);
    out << "\n";
  }

  // member: servo
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "servo: ";
    rosidl_generator_traits::value_to_yaml(msg.servo, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DetectedBox & msg, bool use_flow_style = false)
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

}  // namespace ros2_yolo_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ros2_yolo_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_yolo_msgs::msg::DetectedBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_yolo_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_yolo_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros2_yolo_msgs::msg::DetectedBox & msg)
{
  return ros2_yolo_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_yolo_msgs::msg::DetectedBox>()
{
  return "ros2_yolo_msgs::msg::DetectedBox";
}

template<>
inline const char * name<ros2_yolo_msgs::msg::DetectedBox>()
{
  return "ros2_yolo_msgs/msg/DetectedBox";
}

template<>
struct has_fixed_size<ros2_yolo_msgs::msg::DetectedBox>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2_yolo_msgs::msg::DetectedBox>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2_yolo_msgs::msg::DetectedBox>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_YOLO_MSGS__MSG__DETAIL__DETECTED_BOX__TRAITS_HPP_
