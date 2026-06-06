# Universal GNSS Sidecar Migration

## Goal

Move Universal GNSS out of `mowgli-ros2` and make the existing `gps` container the sole GNSS sidecar.

End-state:

- `mowgli-ros2` does not build `universal_gnss_ros2`
- `mowgli-ros2` does not launch `receiver_node` or `ntrip_node`
- the `gps` container owns:
  - `receiver_node`
  - `ntrip_node`
  - `replay_node` (future)
  - RTCM transport
  - GNSS diagnostics
- `mowgli-ros2` consumes only:
  - `/gps/fix`
  - `/gps/status`
  - `/diagnostics`
  - `/rtcm`

## Recommended Boundary Contract

Use stable, non-Universal-GNSS message types on the inter-container boundary:

- `/gps/fix` -> `sensor_msgs/msg/NavSatFix`
- `/gps/status` -> `mowgli_interfaces/msg/GnssStatus`
- `/diagnostics` -> `diagnostic_msgs/msg/DiagnosticArray`
- `/rtcm` -> `rtcm_msgs/msg/Message`

Reason:

- if `/gps/status` or `/rtcm` use `universal_gnss_ros2/*` types on the shared graph, `mowgli-ros2` still needs those interface packages for Foxglove schema resolution and any typed subscribers
- keeping the boundary on Mowgli or standard ROS messages is the cleanest way to satisfy "consume topics only"

Inside the sidecar, Universal GNSS can still use its native message types and bridge them to the public contract.

## Current State

Today the repo is split across two places:

- `ros2/src/mowgli_bringup/launch/universal_gnss.launch.py` launches `universal_gnss_ros2` from inside `mowgli-ros2`
- `ros2/src/mowgli_bringup/launch/full_system.launch.py` includes that launch when `GNSS_STACK != disabled`
- `ros2/Dockerfile` vendors and builds `ros2/src/external/universal-gnss`
- `ros2/src/mowgli_bringup/package.xml` depends on `universal_gnss_ros2`
- `install/lib/compose.sh` treats `GNSS_STACK=universal` as "no gps sidecar"
- `install/compose/docker-compose.gps.yml` already defines a `mowgli-gps` service, but it is currently the legacy receiver container
- `sensors/gps/` already owns NTRIP, RTCM writeback, and GNSS diagnostics for the current driver stack

That means the architectural split already exists in practice, but ownership is divided across both images.

## 1. Migration Plan

### Phase 1: Establish the sidecar boundary

- keep the public topics exactly the same
- add a sidecar launch or entrypoint in `sensors/gps/` that starts:
  - `universal_gnss_ros2/receiver_node`
  - `universal_gnss_ros2/ntrip_node`
  - a status bridge: Universal GNSS status -> `mowgli_interfaces/msg/GnssStatus`
  - an RTCM bridge: Universal GNSS RTCM -> `rtcm_msgs/msg/Message`
  - diagnostics aggregation or relay
- do not change `mowgli-ros2` behavior yet
- add CI coverage for the new sidecar image path

### Phase 2: Flip runtime ownership to the sidecar

- stop launching Universal GNSS from `mowgli_bringup`
- always include the GNSS sidecar compose fragment for direct-GNSS deployments
- keep `navsat_to_absolute_pose_node` as a consumer of `/gps/fix` only
- keep `publish_gnss_status:=false` in `navsat_to_absolute_pose_node`

### Phase 3: Remove Universal GNSS from `mowgli-ros2`

- remove `universal_gnss_ros2` from `mowgli_bringup/package.xml`
- remove Universal GNSS package copying from `ros2/Dockerfile`
- remove workspace sync logic that links the vendored Universal GNSS package into the main ROS workspace
- remove ROS2 CI and Docker smoke tests that assume `universal_gnss_ros2` exists in the main image

### Phase 4: Collapse backend-specific GNSS containers

- converge `gps` and `unicore` runtime selection onto the single GNSS sidecar
- retain receiver-family selection through env/config:
  - `GNSS_RECEIVER_FAMILY=ublox|unicore|nmea|auto`
- keep receiver-specific configuration helpers only where they are still needed
- retire the separate `unicore` image after parity is proven

## 2. Required Dockerfile Changes

### `sensors/gps/Dockerfile`

Replace the current "legacy driver image" role with "Universal GNSS sidecar" role.

Required changes:

- stop treating `ublox_dgnss` as the primary runtime
- add a builder stage that builds:
  - `universal_gnss_ros2`
  - sidecar bridge code
  - `mowgli_interfaces` if `/gps/status` stays on the Mowgli message contract
- keep runtime packages needed for the public contract:
  - `ros-kilted-rtcm-msgs`
  - `ros-kilted-diagnostic-msgs`
  - `ros-kilted-rmw-cyclonedds-cpp`
- keep any serial/NTRIP helper dependencies that remain relevant
- replace the current `start_gps.sh` process set with the sidecar process set

Recommended implementation detail:

- build the sidecar from repo-root context with `file: sensors/gps/Dockerfile`
- copy in:
  - `ros2/src/external/universal-gnss/`
  - `ros2/src/mowgli_interfaces/`
  - sidecar-specific wrapper package under `sensors/gps/` or a dedicated sidecar package directory

This avoids cloning multiple repos at image-build time and keeps the image tied to the monorepo revision under test.

### `ros2/Dockerfile`

Remove Universal GNSS from the main image:

- delete the copied metadata for:
  - `src/external/universal-gnss/gnss_ros2/package.xml`
  - `src/external/universal-gnss/gnss_ros2/CMakeLists.txt`
- stop building or resolving `universal_gnss_ros2` in the `ros2` workspace
- remove the ROS2 smoke checks that expect the package to exist

### `_sensor-docker.yml`

The reusable sensor workflow likely needs a `dockerfile` input so the `gps` image can build from repo root while still using `sensors/gps/Dockerfile`.

## 3. Required Compose Changes

### `install/compose/docker-compose.gps.yml`

Make this the canonical direct-GNSS sidecar fragment.

Required changes:

- keep service name and container name stable:
  - service: `gps`
  - container: `mowgli-gps`
- pass the normalized GNSS contract into the container:
  - `GNSS_RECEIVER_FAMILY`
  - `GNSS_TRANSPORT`
  - `GNSS_SERIAL_DEVICE`
  - `GNSS_SERIAL_BAUD`
  - `GNSS_NTRIP_*`
- keep compatibility env passthrough for one transition window:
  - `GPS_PROTOCOL`
  - `GPS_PORT`
  - `GPS_BAUD`
  - `GPS_BY_ID`

### `install/compose/docker-compose.unicore.yaml`

Two reasonable options:

- short term: keep it, but repoint it to the same sidecar image and only vary env/config
- end state: remove it and let `GNSS_RECEIVER_FAMILY=unicore` select the receiver path inside `docker-compose.gps.yml`

### `install/lib/compose.sh`

This file is a key behavior change.

Required changes:

- stop treating `GNSS_STACK=universal` as "GNSS runs inside mowgli-ros2"
- include the GNSS compose fragment whenever `effective_gnss_backend != disabled`
- normalize `legacy` and `universal` onto the same sidecar path during transition
- update log messages to say GNSS runs in `mowgli-gps`, not `mowgli-ros2`

### `install/lib/checks.sh`

Required changes:

- invert the current universal-mode expectation
- in direct-GNSS mode, `mowgli-gps` should be required, not flagged as wrong
- `/gps/fix` and `/rtcm` health checks should point operators to `mowgli-gps` logs first

## 4. Required Launch Changes

### Remove GNSS launch ownership from `mowgli_bringup`

Files:

- `ros2/src/mowgli_bringup/launch/universal_gnss.launch.py`
- `ros2/src/mowgli_bringup/launch/full_system.launch.py`
- `ros2/src/mowgli_bringup/package.xml`

Required changes:

- delete or deprecate `universal_gnss.launch.py`
- remove the `use_universal_gnss` argument from `full_system.launch.py`
- remove the `IncludeLaunchDescription(...)` for Universal GNSS from `full_system.launch.py`
- update comments and docstrings to state GNSS is external to `mowgli-ros2`
- remove the `universal_gnss_ros2` exec dependency from `mowgli_bringup/package.xml`

### Keep localization consuming the boundary only

Files:

- `ros2/src/mowgli_bringup/launch/full_system.launch.py`
- `ros2/src/mowgli_bringup/launch/sim_full_system.launch.py`
- `ros2/src/mowgli_localization/src/navsat_to_absolute_pose_node.cpp`

Expected state:

- `navsat_to_absolute_pose_node` continues to consume `/gps/fix`
- local `/gps/status` publishing stays disabled in normal runtime launch
- simulation keeps its current fake/sim GNSS path and should not depend on Universal GNSS packages either

## 5. CI Impact

### ROS2 CI and ROS2 Docker

Files:

- `.github/workflows/ros2-ci.yml`
- `.github/workflows/ros2-docker.yml`
- `ros2/Makefile`
- `ros2/scripts/sync_workspace_packages.sh`
- `ros2/README.md`

Required changes:

- remove validation that `universal_gnss_ros2` exists in `mowgli-ros2`
- remove smoke tests that run `ros2 launch mowgli_bringup universal_gnss.launch.py`
- remove `universal_gnss_ros2` from `ros2/Makefile` development package defaults
- stop linking the vendored Universal GNSS package into the main workspace sync path
- update build docs that currently describe Universal GNSS as part of the main ROS2 workspace

### Sensor CI

Files:

- `.github/workflows/sensors-gps.yml`
- `.github/workflows/_sensor-docker.yml`

Required changes:

- add a sidecar smoke test in the `gps` image build:
  - package presence
  - launch or executable availability
  - bridge package availability
- if repo-root build context is adopted, pass both:
  - `context: .`
  - `dockerfile: sensors/gps/Dockerfile`

### Installer and shell tests

Files under:

- `install/test_mowglinext.sh`
- `install/tests/`

Expected updates:

- compose selection expectations
- `GNSS_STACK=universal` behavior
- health-check messages
- any assertions that universal mode means "no mowgli-gps container"

## 6. Backward Compatibility Strategy

### Keep the user-facing runtime API stable

- keep topic names unchanged
- keep `mowgli-gps` container name unchanged
- keep `GNSS_BACKEND`, `GPS_*`, and `GNSS_*` env keys accepted
- translate legacy env/config into the normalized sidecar env contract

### Deprecate, do not hard-break

- accept `GNSS_STACK=legacy` for one or more releases
- internally normalize both `legacy` and `universal` onto the sidecar architecture
- log a deprecation warning when `legacy` is used

### Preserve the `/gps/status` type if possible

Best option:

- keep `/gps/status` as `mowgli_interfaces/msg/GnssStatus`
- do the Universal GNSS -> Mowgli status adaptation inside the sidecar

This avoids touching:

- Foxglove schema expectations in `gui/pkg/providers/ros.go`
- any downstream code that assumes the Mowgli GNSS status schema

### Preserve `/rtcm` as a standard ROS message

Best option:

- expose `/rtcm` as `rtcm_msgs/msg/Message`
- keep any richer Universal GNSS RTCM representation internal to the sidecar

## 7. Minimal First PR

The safest first PR is a prep PR, not the final cutover.

Scope:

- add a Universal GNSS sidecar launcher inside `sensors/gps/`
- add bridge nodes so the sidecar can publish the stable public contract:
  - `/gps/fix`
  - `/gps/status`
  - `/diagnostics`
  - `/rtcm`
- update `sensors-gps` CI to build and smoke-test that new path
- do not remove the in-process `mowgli-ros2` launch path yet
- do not change installer defaults yet

Why this is the right first PR:

- it proves the sidecar can produce the exact boundary topics before any compose flip
- it lets CI validate the new image independently
- it keeps rollback trivial
- it avoids a giant "move runtime, remove dependency, rewrite installer, rewrite CI" PR

## Suggested PR Sequence

1. Prep PR
   - build Universal GNSS in `sensors/gps`
   - add sidecar launch and topic bridges
   - add sensor CI smoke tests
2. Runtime flip PR
   - switch compose and installer to always run `mowgli-gps` for direct GNSS
   - stop launching GNSS from `mowgli_bringup`
3. Cleanup PR
   - remove Universal GNSS from `ros2/Dockerfile`, `package.xml`, Makefile, docs, and ROS2 CI
4. Convergence PR
   - collapse `unicore` onto the shared sidecar and remove duplicate container logic
