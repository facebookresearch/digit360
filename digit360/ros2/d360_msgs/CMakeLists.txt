# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

cmake_minimum_required(VERSION 3.8)
project(d360_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add custom message files
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/AudioDataD360.msg
  msg/AudioInfoD360.msg
  msg/GasHTData.msg
  msg/ImuRawD360.msg
  msg/ImuEulerD360.msg
  msg/ImuQuatD360.msg
  msg/PressureAPD360.msg
  msg/PressureD360.msg
  msg/LightControl.msg
  
  DEPENDENCIES geometry_msgs std_msgs

)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
