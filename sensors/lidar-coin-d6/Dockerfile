# =============================================================================
# WitMotion Coin-D6 LiDAR driver for ROS2 Kilted (cspc_lidar / Guoke "COIN" core)
#
# Source confirmed (by direct diff against the seller-provided Coin-D6 driver)
# to share an identical wire protocol -- serial_port.cpp and
# lidar_data_processing.cpp are byte-for-byte the same. This package is kept
# instead of the seller's because it has safe parameter defaults (the seller's
# version crashes on startup if every one of port/baudrate/frame_id/version
# isn't supplied externally) and consistent node-name/param-key matching.
#
# Stripped vs. the original cspc_lidar repo: no PCL/PointCloud2 publisher, no
# dead calibration module, no unused control/error topics -- see
# src/node_lidar_ros.cpp for details. Publishes sensor_msgs/LaserScan on
# /scan with frame_id=lidar_link, matching every other LiDAR driver in this
# repo (lidar-ldlidar, lidar-rplidar, lidar-stl27l).
# =============================================================================
FROM ros:kilted-ros-base AS build

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-kilted-ament-cmake \
    build-essential \
    cmake \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src/cspc_lidar
COPY CMakeLists.txt package.xml ./
COPY include ./include
COPY sdk ./sdk
COPY src ./src

WORKDIR /ros2_ws
RUN /bin/bash -c "\
    source /opt/ros/kilted/setup.bash && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      --parallel-workers \$(nproc) \
    "

# =============================================================================
# Runtime — minimal image
# =============================================================================
FROM ros:kilted-ros-base AS runtime

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kilted-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY --from=build /ros2_ws/install/ install/

COPY ros2_entrypoint.sh /ros2_entrypoint.sh
RUN chmod +x /ros2_entrypoint.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["ros2", "run", "cspc_lidar", "cspc_lidar", \
     "--ros-args", \
     "-p", "port:=/dev/lidar", \
     "-p", "baudrate:=230400", \
     "-p", "frame_id:=lidar_link", \
     "-p", "version:=4"]
