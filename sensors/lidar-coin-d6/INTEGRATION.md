# sensors/lidar-coin-d6/ — integration notes

Base: `cspc_lidar` (vendored COIN-D4A ROS2 driver). Confirmed by direct source
diff to be wire-protocol-identical to the seller's Coin-D6 driver
(`serial_port.cpp` / `lidar_data_processing.cpp` are byte-for-byte the same).
This package was reworked to be as close as possible in shape and footprint
to the other three LiDAR drivers already in this repo
(`sensors/lidar-ldlidar/`, `sensors/lidar-rplidar/`, `sensors/lidar-stl27l/`):
single `ros2 run ... --ros-args -p ...` CMD, no mounted params file, no
launch tree, minimal runtime image with only `ros-kilted-rmw-cyclonedds-cpp`
added, `/scan` + `frame_id=lidar_link` as the only output.

## Everything removed, and why

| Removed | Why |
|---|---|
| `sdk/calibration.cpp/.h` | Dead code. Its only call site in `node_lidar.cpp` was already commented out upstream, gated behind a flag (`data_calibration`) that defaulted to `false` and was never set anywhere. Also dropped the now-unused `data_calibration` field from `node_lidar.h`. |
| `sdk/main.cpp` | Not referenced by the top-level `CMakeLists.txt`'s build target at all (leftover non-ROS CLI entry point superseded by `src/node_lidar_ros.cpp`'s `main()`). |
| `sdk/CMakeLists.txt` (the inner one) | Orphaned sub-CMakeLists, never `add_subdirectory`'d by the top-level build. |
| PCL / `pcl_conversions` / `/point_cloud` (PointCloud2) publisher | Pulled in `libpcl-dev` + `pcl_conversions` — heavy, and a portability risk in a multi-arch Docker build — to publish a topic nothing in MowgliNext consumes. Nav2, collision_monitor, and obstacle_tracker_node only ever read `/scan`. |
| `visualization_msgs`, `geometry_msgs` deps | Grepped the entire source tree — neither is referenced anywhere in any `.cpp`/`.h`. Leftover `ros2 pkg create` template boilerplate. |
| `rosidl_default_generators/runtime`, `fastcdr`, `member_of_group rosidl_interface_packages`, `src/Error.msg` | The custom `Error` message is never actually compiled — there's no `rosidl_generate_interfaces()` call anywhere in `CMakeLists.txt`. Pure dead weight. |
| `lidar_status` (UInt16) control subscriber + `MinimalSubscriber` node, `lsd_error` (String) error topic | None of the other 3 MowgliNext LiDAR drivers expose any control/error topic — they're plain `/scan` publishers. The Coin-D6 motor auto-starts regardless of whether anything publishes to `lidar_status`, so removing the subscriber changes nothing about normal operation; you only lose the ability to runtime-toggle exposure/speed/start-stop via a topic nobody was using. |
| `rviz/`, `launch/`, `params/` (whole dirs) | Not used headless. The other 3 drivers configure entirely via Docker `CMD` args; matched that pattern instead of shipping a mounted YAML + launch tree. |
| `-march=native -mtune=generic -O0 -g` + `CMAKE_BUILD_TYPE Debug` | `-march=native` bakes in build-machine CPU instructions — breaks portability across a multi-arch (amd64/arm64) Docker build. Switched to a normal optimized `Release` build, matching every other sensor Dockerfile in this repo. |

What's left is exactly: serial port handling, packet parsing
(`lidar_data_processing.cpp`), point filtering / blocked-lidar detection
(`point_cloud_optimize.cpp` — this is internal driver logic, unrelated to the
external PCL library despite the similar name), and a thin `main()` that
publishes `sensor_msgs/LaserScan` on `/scan`.

## Defaults changed (vs. upstream cspc_lidar)

| Param | Upstream default | Changed to | Why |
|---|---|---|---|
| `port` | `/dev/sc_mini` | `/dev/lidar` | MowgliNext's installer (`install/lib/lidar.sh`) generates a udev rule that always symlinks the configured LiDAR to `/dev/lidar`, regardless of brand — matches the convention `lidar-ldlidar`/`lidar-rplidar`/`lidar-stl27l` all use. |
| `frame_id` | `"Null"` | `lidar_link` | Matches `mowgli_bringup`'s URDF and every other LiDAR driver in this repo. |
| `version` | `0` | `4` | `4` = `M1CT_TOF` in this codebase — confirmed identical numeric value and parsing path to the seller driver's `M1CT_Coin_D2` setting. `0` would hit the `default: break;` case in `initialize()` and never configure a baudrate/sample format. |
| `baudrate` | `115200` | `230400` | Coin-D6's actual UART rate (same as LD19/STL27L). |

## Steps to wire it in

1. Copy `lidar-coin-d6/` into your checkout as `sensors/lidar-coin-d6/`.
2. Copy `docker-compose.lidar-coin-d6.yml` into `install/compose/`.
3. (Optional, recommended) Copy `sensors-lidar-coin-d6.yml` into
   `.github/workflows/` so CI builds and publishes the image the same way it
   does for the other 3 LiDAR variants.
4. **udev rule on the host** — none of the 3 existing drivers ship a udev
   rules file in their `sensors/` folder; MowgliNext's installer generates one
   dynamically based on whatever serial device you pick. Since the Coin-D6
   connects via a USB-serial chip (not raw UART), the simplest robust rule is
   to match by USB vendor/product ID (check yours with `lsusb` after plugging
   it in — common ones are CH340 `1a86:7523` or CP2102 `10c4:ea60`):
   ```bash
   echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="lidar", MODE="0666"' \
     | sudo tee /etc/udev/rules.d/99-mowgli-lidar.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
   Adjust the vendor/product pair if `lsusb` shows something else.
5. In `docker/.env`, set:
   ```
   LIDAR_IMAGE=<your build tag, or ghcr.io/<you>/mowglinext/lidar-coin-d6:main once CI publishes it>
   LIDAR_PORT=/dev/lidar
   LIDAR_BAUD=230400
   ```
6. Build + smoke test:
   ```bash
   docker compose -f install/compose/docker-compose.lidar-coin-d6.yml build lidar
   docker compose -f install/compose/docker-compose.lidar-coin-d6.yml up lidar
   # separately, same ROS2 domain:
   ros2 topic hz /scan
   ros2 topic echo /scan --once
   ```
   Check `header.frame_id == lidar_link` and that `range_min`/`range_max`
   look sane. Note the driver hardcodes `range_min=0.10`/`range_max=10.0` in
   `send_lidar_data()` regardless of any `max_range` you might try to pass —
   that parameter doesn't exist on this stripped node at all anymore (it was
   never wired to anything upstream either).
7. Not done here, left for you: wiring `coin-d6` in as a 5th menu choice in
   `install/lib/lidar.sh` (currently only offers none/rplidar/ldlidar/stl27l)
   and a matching case in `install/tests/test_lidar_matrix.sh`. Say the word
   if you want that patched too.

## License note

Upstream `package.xml` had `<license>TODO: License declaration</license>` —
the vendor never specified one. Fine for your own private fork; worth
sorting out before publishing this driver anywhere public.
