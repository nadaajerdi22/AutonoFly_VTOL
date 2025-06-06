// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/PwmInput.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "px4_msgs/msg/pwm_input.h"


#ifndef PX4_MSGS__MSG__DETAIL__PWM_INPUT__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__PWM_INPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/PwmInput in the package px4_msgs.
typedef struct px4_msgs__msg__PwmInput
{
  /// Time since system start (microseconds)
  uint64_t timestamp;
  /// Timer overcapture error flag (AUX5 or MAIN5)
  uint64_t error_count;
  /// Pulse width, timer counts (microseconds)
  uint32_t pulse_width;
  /// Period, timer counts (microseconds)
  uint32_t period;
} px4_msgs__msg__PwmInput;

// Struct for a sequence of px4_msgs__msg__PwmInput.
typedef struct px4_msgs__msg__PwmInput__Sequence
{
  px4_msgs__msg__PwmInput * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__PwmInput__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__PWM_INPUT__STRUCT_H_
