// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_yolo_msgs:msg/DetectedBox.idl
// generated code does not contain a copyright notice

#ifndef ROS2_YOLO_MSGS__MSG__DETAIL__DETECTED_BOX__STRUCT_H_
#define ROS2_YOLO_MSGS__MSG__DETAIL__DETECTED_BOX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/DetectedBox in the package ros2_yolo_msgs.
typedef struct ros2_yolo_msgs__msg__DetectedBox
{
  float x1;
  float y1;
  float x2;
  float y2;
  int32_t servo;
} ros2_yolo_msgs__msg__DetectedBox;

// Struct for a sequence of ros2_yolo_msgs__msg__DetectedBox.
typedef struct ros2_yolo_msgs__msg__DetectedBox__Sequence
{
  ros2_yolo_msgs__msg__DetectedBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_yolo_msgs__msg__DetectedBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_YOLO_MSGS__MSG__DETAIL__DETECTED_BOX__STRUCT_H_
