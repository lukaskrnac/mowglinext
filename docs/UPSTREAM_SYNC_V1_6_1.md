# Fork synchronization with MowgliNext v1.6.1

## Source baseline

- Fork: `lukaskrnac/mowglinext`, `sync/upstream-v1.5.0`, commit
  `0ada4a26732a9b6edcf7c3c8276163ca474cf1c0`.
- Upstream release: `mowglinext/mowglinext`, `v1.6.1`, commit
  `fe77a09715f38de571b929e7b7aa567212295bd8`.
- The merge retains both parents and preserves the fork's external LiDAR primary
  localization, LiDAR map calibration, GLIM controls, source-aware localization
  guards, and configurable manual driving speed.

## Conflict resolutions

Eight GUI files needed manual resolution. Both sets of topic registrations and
parameter helpers are retained. The localization settings retain upstream's
schema-derived types, and the dashboard localization row has the new stable id.
Manual driving uses upstream's gesture ownership, WebSocket generation checks,
immediate STOP handling and nonqueued Twist messages, with the fork's configured
linear speed cap applied when constructing motion messages. Regression tests
exercise the configured cap together with reconnect and STOP behavior.
The settings types are regenerated from the merged schema.

## Hardware acceptance — HARDWARE_REQUIRED

No physical mower, ESP32 image, STM32 image, sensor-driver revision or external
GLIM/localizer checkout was available in the build environment. No hardware
compatibility or field-performance claim is made by this merge.

The upstream bridge now expects protocol v9, compared with v7 in the source
fork. In particular, `LlOdometry` changes from 17 to 19 bytes:
`uint16_t dt_millis` becomes `uint32_t acquisition_millis`. The standard STM32
backend requires matching firmware. A custom ESP32 implementation needs its
actual wire format and handshake checked; changing only its advertised protocol
number does not establish compatibility. The fork does not include that ESP32
firmware, so this synchronization does not port or flash it. The upstream
OpenMower backend has a separate protocol and update path.

Before testing, record the final merge commit, actual firmware image hash,
hardware backend, robot identity, external GLIM/localizer commit and sensor-driver
revision. Back up installed configuration, map geometry, LiDAR calibration,
GLIM map and firmware. Perform the following checks on that exact baseline:

1. Disconnect blade motor power, lift the drive wheels clear of the ground, and
   keep an immediately accessible physical emergency stop. Verify the bridge
   accepts the genuine firmware protocol and decodes monotonic odometry, wheel
   direction and elapsed acquisition time correctly, including dropped packets.
   A mismatch, malformed odometry or unexpected actuation is a failure.
2. In manual mode, check configured speed, joystick release, explicit STOP,
   navigation away, mode transitions and loss/reconnection of the GUI socket.
   STOP/release must produce zero commands; reconnect must not resume an old
   motion gesture. Confirm physical e-stop independently.
3. With blade power still disconnected, switch between GPS and calibrated LiDAR.
   Confirm the active source, continuous TF chain, valid map-frame pose and
   working GLIM/calibration status. On invalid/stale LiDAR input, the existing
   localization guards must refuse or stop the operation as designed. Confirm
   LiDAR-primary operation without GNSS rather than assuming a successful merge
   proves it.
4. Only after the bench checks pass, conduct a supervised low-speed drive in a
   cleared bounded area with blade power disconnected. Verify localization,
   boundary/collision handling and immediate stop on the recorded baseline.
   Validate blade readiness/telemetry and blade stop separately under the
   hardware's established safe test procedure before any mowing acceptance.

A full ROS2 build and the above hardware evidence remain separate from the
GUI/backend and static launch/configuration checks.
