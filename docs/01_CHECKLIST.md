# Checklist — aplikácia do forku

## A. Offline, raz
- [ ] `tools/align_glim_map.py` — zarovnaj GLIM `.ply` do ENU rámca
      `fusion_graph`u (dock = (0,0), sever = +y). Over vizuálne v RViz
      (prekry s GPS trajektóriou).

## B. `fusion_graph` package (fork)
- [ ] Aplikuj `fusion_graph_patch/fusion_graph_node.hpp.patch`
      (`patch -p1 < fusion_graph_node.hpp.patch` z koreňa `fusion_graph/`,
      alebo ručne — je to len pridávanie, žiadne mazanie existujúceho kódu).
- [ ] Skopíruj `fusion_graph_patch/fusion_graph_node_lidar_primary.cpp` do
      `src/`.
- [ ] Aplikuj `fusion_graph_node_setup_params.cpp.patch`.
- [ ] Aplikuj `fusion_graph_node_setup_comms.cpp.patch`.
- [ ] Aplikuj `fusion_graph_node_callbacks_a.cpp.patch`.
- [ ] Aplikuj `CMakeLists.txt.patch`.
- [ ] Aplikuj `fusion_graph.yaml.patch` (alebo pridaj rovnaké kľúče do
      svojho nasadeného `mowgli_robot.yaml`/prekryvu, podľa toho, ako u
      vás funguje deep-merge cez `robot_config_util.py`).
- [ ] `colcon build --packages-select fusion_graph` — over kompiláciu.
      **Nekontrolované tu**: nemám k dispozícii plný workspace s GTSAM/
      Beluga/Sophus hlavičkami, takže toto som nemohol reálne skompilovať.
      Prejdi kód, priezvi typy (`rclcpp::Parameter`, `OnSetParametersCallbackHandle`
      require `#include <rclcpp/rclcpp.hpp>` — už je included).

## C. Bringup
- [ ] Priprav `lidar_localization_ros2` launch podľa
      `bringup/lidar_localization_bringup.launch.py` — **uprav
      package/executable/parameter názvy podľa toho, čo máš aktuálne
      naklonované** (ich launch API sa podľa README aktívne mení release
      od release — over `ros2 launch lidar_localization_ros2
      nav2_lidar_localization.launch.py --show-args` alebo si pozri ich
      aktuálny launch súbor priamo).
- [ ] **Kriticky over a vypni ich TF broadcast** (nech `fusion_graph`
      ostáva jediný, kto publikuje `map→odom`/`odom→base_footprint`).
- [ ] Priprav `pointcloud_to_laserscan` most (`bringup/pointcloud_to_laserscan.launch.py`)
      pre existujúci Nav2 costmap/collision pipeline z VLP16 3D mračna.
- [ ] Skontroluj `lidar_pose_topic`/`lidar_alignment_status_topic` v
      `fusion_graph.yaml` sedia s reálnymi topicmi z
      `lidar_localization_ros2`.

## D. Overenie v teréni
- [ ] Spusti všetko s `primary_localization_source: gps` (default,
      nezmenené správanie) a sleduj `/fusion_graph/diagnostics` +
      `/alignment_status` — LiDAR beží, počíta kvalitu, ale nefunduje do
      grafu. Over že `failure_category=healthy` väčšinu času.
  - [ ] Prepni naživo: `ros2 param set /fusion_graph_node
        primary_localization_source lidar`. Sleduj plynulý prechod (žiadny
        skok v `map→odom`), `anchor_slew_enabled` limiter by mal doladiť
        akýkoľvek malý rozdiel medzi zdrojmi hladko.
  - [ ] Otestuj prepnutie späť na `gps` a znova na `lidar` niekoľkokrát.
  - [ ] Over `lidar_pose_feed_yaw` — sleduj `cov_yawyaw` v diagnostike,
        porovnaj s COG/mag yaw počas known-good úsekov.

## Vedomé zjednodušenia v tomto v1 (čo dorobiť neskôr)
- Žiadny LiDAR-špecifický "wrong-fix" jump gate (mirror
  `rtk_wrongfix_gate.hpp`) — spolieha sa na `/alignment_status`
  (`failure_category`, `consecutive_rejected_updates`) + kovariančný reject.
  Ak sa v teréni ukáže, že toto nestačí (NDT sa niekedy "chytí" na zlom
  lokálnom minime s dobrým fitness skóre), pridaj analogickú kontrolu proti
  `wheel_dist_since_last_*`/`abs_dtheta_since_last_*` — ale **nezdieľaj**
  akumulátory s GPS cestou (viedlo by to k falošným odmietnutiam hneď po
  prepnutí zdroja).
- `primary_localization_source` je binárny prepínač (buď/alebo). Continuum
  "oba naraz, nech ich kovariancie rozhodnú váhu" (podobne ako dnes
  koexistujú GPS a gyro/COG) je architektonicky jednoduchšie dosiahnuteľné
  (stačí odstrániť `if (!primary_is_lidar_...)`/`if (primary_is_lidar_...)`
  gaty), ale zámerne som to nechal ako tvrdý prepínač — presne to, čo si
  žiadal ako prvý krok, a je to bezpečnejšie na ladenie (vieš si byť istý,
  ktorý zdroj práve vidíš).
