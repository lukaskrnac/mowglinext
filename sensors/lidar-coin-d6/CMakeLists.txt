cmake_minimum_required(VERSION 3.5)

project(cspc_lidar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# NOTE: upstream vendor CMakeLists.txt set "-march=native -mtune=generic -O0
# -g" + CMAKE_BUILD_TYPE=Debug. -march=native bakes in build-machine-specific
# CPU instructions, which breaks portability across host CPUs and especially
# across amd64/arm64 multi-arch Docker builds. Use a normal optimized Release
# build instead, matching the other MowgliNext sensor containers.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include/cspc_lidar
  ${PROJECT_SOURCE_DIR}/sdk
)

# NOTE: dropped sdk/calibration.cpp (dead code -- its only call site in
# node_lidar.cpp was already commented out upstream, gated behind a flag
# that defaulted to false) and the PCL/pcl_conversions dependency that the
# old src/node_lidar_ros.cpp pulled in just to publish an unused /point_cloud
# topic. visualization_msgs and geometry_msgs are also dropped: grepping the
# full source tree shows neither is referenced anywhere, they were leftover
# `ros2 pkg create` template boilerplate.
add_executable(cspc_lidar
  sdk/mtime.cpp
  sdk/node_lidar.cpp
  sdk/serial_port.cpp
  sdk/timer.cpp
  sdk/lidar_data_processing.cpp
  sdk/point_cloud_optimize.cpp
  src/node_lidar_ros.cpp
)

ament_target_dependencies(cspc_lidar
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS
  cspc_lidar
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
