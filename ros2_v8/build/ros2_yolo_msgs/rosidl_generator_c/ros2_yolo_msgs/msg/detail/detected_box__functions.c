// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_yolo_msgs:msg/DetectedBox.idl
// generated code does not contain a copyright notice
#include "ros2_yolo_msgs/msg/detail/detected_box__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ros2_yolo_msgs__msg__DetectedBox__init(ros2_yolo_msgs__msg__DetectedBox * msg)
{
  if (!msg) {
    return false;
  }
  // x1
  // y1
  // x2
  // y2
  // servo
  return true;
}

void
ros2_yolo_msgs__msg__DetectedBox__fini(ros2_yolo_msgs__msg__DetectedBox * msg)
{
  if (!msg) {
    return;
  }
  // x1
  // y1
  // x2
  // y2
  // servo
}

bool
ros2_yolo_msgs__msg__DetectedBox__are_equal(const ros2_yolo_msgs__msg__DetectedBox * lhs, const ros2_yolo_msgs__msg__DetectedBox * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x1
  if (lhs->x1 != rhs->x1) {
    return false;
  }
  // y1
  if (lhs->y1 != rhs->y1) {
    return false;
  }
  // x2
  if (lhs->x2 != rhs->x2) {
    return false;
  }
  // y2
  if (lhs->y2 != rhs->y2) {
    return false;
  }
  // servo
  if (lhs->servo != rhs->servo) {
    return false;
  }
  return true;
}

bool
ros2_yolo_msgs__msg__DetectedBox__copy(
  const ros2_yolo_msgs__msg__DetectedBox * input,
  ros2_yolo_msgs__msg__DetectedBox * output)
{
  if (!input || !output) {
    return false;
  }
  // x1
  output->x1 = input->x1;
  // y1
  output->y1 = input->y1;
  // x2
  output->x2 = input->x2;
  // y2
  output->y2 = input->y2;
  // servo
  output->servo = input->servo;
  return true;
}

ros2_yolo_msgs__msg__DetectedBox *
ros2_yolo_msgs__msg__DetectedBox__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_yolo_msgs__msg__DetectedBox * msg = (ros2_yolo_msgs__msg__DetectedBox *)allocator.allocate(sizeof(ros2_yolo_msgs__msg__DetectedBox), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_yolo_msgs__msg__DetectedBox));
  bool success = ros2_yolo_msgs__msg__DetectedBox__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_yolo_msgs__msg__DetectedBox__destroy(ros2_yolo_msgs__msg__DetectedBox * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_yolo_msgs__msg__DetectedBox__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_yolo_msgs__msg__DetectedBox__Sequence__init(ros2_yolo_msgs__msg__DetectedBox__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_yolo_msgs__msg__DetectedBox * data = NULL;

  if (size) {
    data = (ros2_yolo_msgs__msg__DetectedBox *)allocator.zero_allocate(size, sizeof(ros2_yolo_msgs__msg__DetectedBox), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_yolo_msgs__msg__DetectedBox__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_yolo_msgs__msg__DetectedBox__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ros2_yolo_msgs__msg__DetectedBox__Sequence__fini(ros2_yolo_msgs__msg__DetectedBox__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ros2_yolo_msgs__msg__DetectedBox__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ros2_yolo_msgs__msg__DetectedBox__Sequence *
ros2_yolo_msgs__msg__DetectedBox__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_yolo_msgs__msg__DetectedBox__Sequence * array = (ros2_yolo_msgs__msg__DetectedBox__Sequence *)allocator.allocate(sizeof(ros2_yolo_msgs__msg__DetectedBox__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_yolo_msgs__msg__DetectedBox__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_yolo_msgs__msg__DetectedBox__Sequence__destroy(ros2_yolo_msgs__msg__DetectedBox__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_yolo_msgs__msg__DetectedBox__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_yolo_msgs__msg__DetectedBox__Sequence__are_equal(const ros2_yolo_msgs__msg__DetectedBox__Sequence * lhs, const ros2_yolo_msgs__msg__DetectedBox__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_yolo_msgs__msg__DetectedBox__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_yolo_msgs__msg__DetectedBox__Sequence__copy(
  const ros2_yolo_msgs__msg__DetectedBox__Sequence * input,
  ros2_yolo_msgs__msg__DetectedBox__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_yolo_msgs__msg__DetectedBox);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_yolo_msgs__msg__DetectedBox * data =
      (ros2_yolo_msgs__msg__DetectedBox *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_yolo_msgs__msg__DetectedBox__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_yolo_msgs__msg__DetectedBox__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_yolo_msgs__msg__DetectedBox__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
