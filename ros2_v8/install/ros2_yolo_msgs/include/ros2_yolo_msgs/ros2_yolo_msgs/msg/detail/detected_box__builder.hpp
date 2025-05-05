// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_yolo_msgs:msg/DetectedBox.idl
// generated code does not contain a copyright notice

#ifndef ROS2_YOLO_MSGS__MSG__DETAIL__DETECTED_BOX__BUILDER_HPP_
#define ROS2_YOLO_MSGS__MSG__DETAIL__DETECTED_BOX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_yolo_msgs/msg/detail/detected_box__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_yolo_msgs
{

namespace msg
{

namespace builder
{

class Init_DetectedBox_servo
{
public:
  explicit Init_DetectedBox_servo(::ros2_yolo_msgs::msg::DetectedBox & msg)
  : msg_(msg)
  {}
  ::ros2_yolo_msgs::msg::DetectedBox servo(::ros2_yolo_msgs::msg::DetectedBox::_servo_type arg)
  {
    msg_.servo = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_yolo_msgs::msg::DetectedBox msg_;
};

class Init_DetectedBox_y2
{
public:
  explicit Init_DetectedBox_y2(::ros2_yolo_msgs::msg::DetectedBox & msg)
  : msg_(msg)
  {}
  Init_DetectedBox_servo y2(::ros2_yolo_msgs::msg::DetectedBox::_y2_type arg)
  {
    msg_.y2 = std::move(arg);
    return Init_DetectedBox_servo(msg_);
  }

private:
  ::ros2_yolo_msgs::msg::DetectedBox msg_;
};

class Init_DetectedBox_x2
{
public:
  explicit Init_DetectedBox_x2(::ros2_yolo_msgs::msg::DetectedBox & msg)
  : msg_(msg)
  {}
  Init_DetectedBox_y2 x2(::ros2_yolo_msgs::msg::DetectedBox::_x2_type arg)
  {
    msg_.x2 = std::move(arg);
    return Init_DetectedBox_y2(msg_);
  }

private:
  ::ros2_yolo_msgs::msg::DetectedBox msg_;
};

class Init_DetectedBox_y1
{
public:
  explicit Init_DetectedBox_y1(::ros2_yolo_msgs::msg::DetectedBox & msg)
  : msg_(msg)
  {}
  Init_DetectedBox_x2 y1(::ros2_yolo_msgs::msg::DetectedBox::_y1_type arg)
  {
    msg_.y1 = std::move(arg);
    return Init_DetectedBox_x2(msg_);
  }

private:
  ::ros2_yolo_msgs::msg::DetectedBox msg_;
};

class Init_DetectedBox_x1
{
public:
  Init_DetectedBox_x1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectedBox_y1 x1(::ros2_yolo_msgs::msg::DetectedBox::_x1_type arg)
  {
    msg_.x1 = std::move(arg);
    return Init_DetectedBox_y1(msg_);
  }

private:
  ::ros2_yolo_msgs::msg::DetectedBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_yolo_msgs::msg::DetectedBox>()
{
  return ros2_yolo_msgs::msg::builder::Init_DetectedBox_x1();
}

}  // namespace ros2_yolo_msgs

#endif  // ROS2_YOLO_MSGS__MSG__DETAIL__DETECTED_BOX__BUILDER_HPP_
