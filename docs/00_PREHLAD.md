# LiDAR ako primárna lokalizácia vo fusion_graph — prehľad zmien

Cieľ: `lidar_localization_ros2` (NDT proti vašej GLIM `.pcd`/`.ply` mape) beží
**90 % času ako primárny zdroj absolútnej XY korekcie** v `fusion_graph`u.
GPS ostáva pripojený a naďalej beží (freshness/dock/re-anchor logika sa
nemení), ale jeho príspevok do grafu (`QueueGnss`) sa dá manuálne vypnúť
prepínačom. Wheel+gyro dead-reckoning (`odom→base_footprint`) sa nemení vôbec
— to rieši `fusion_graph` už dnes nezávisle od toho, odkiaľ berie absolútnu
korekciu.

Nepoužívame vstavaný `use_lidar_map_anchor` (Beluga 2D AMCL) — ten je
naschvál len fallback na krátky GNSS výpadok, s vlastnou 2D occupancy grid a
XY-only korekciou po desiatkach sekúnd bez GPS. Namiesto toho pridávame
**nový, nezávislý vstupný kanál**, ktorý používa presne tú istú, už
existujúcu a odladenú cestu do grafu (`GraphManager::QueueLidarMapXy`,
prípadne aj `QueueYaw`), ktorú `GraphManager` nerozlišuje podľa toho, či ju
zavolal vstavaný Beluga filter, alebo váš vonkajší NDT localizer. Táto
metóda **nie je** podmienená flagom `use_lidar_map_anchor` — je to čistá
GraphManager API, takže žiadny zásah do GTSAM/iSAM2 vrstvy nie je potrebný.

## Čo je nové

1. **Nová translation unit** `src/fusion_graph_node_lidar_primary.cpp`
   (+ deklarácie v `fusion_graph_node.hpp`): odoberá `/pcl_pose`
   (`geometry_msgs/PoseWithCovarianceStamped`, plná 6×6 kovariancia) a
   `/alignment_status` (`diagnostic_msgs/DiagnosticArray`) z
   `lidar_localization_ros2`, validuje kvalitu a volá
   `graph_->QueueLidarMapXy(...)` (+ voliteľne `QueueYaw(...)`).
2. **Manuálny prepínač** — ROS2 parameter `primary_localization_source`
   (`"gps"` / `"lidar"`), nastaviteľný za behu (`ros2 param set`), gatuje
   ktorý zdroj sa reálne fúzuje do grafu. Druhý zdroj naďalej beží a počíta
   diagnostiku (aby ste videli kvalitu pred prepnutím), len sa nevolá jeho
   `Queue*`.
3. **Malá úprava `OnGnss`** (existujúci súbor) — jedna podmienka navyše pred
   `graph_->QueueGnss(...)`.
4. **Nové parametre** v `config/fusion_graph.yaml`.
5. **CMakeLists.txt** — pridanie nového `.cpp` do `add_executable`.
6. **Bringup launch** pre `lidar_localization_ros2` v Nav2 móde + prevod
   3D VLP16 mračna na 2D `/scan` pre existujúci Nav2 costmap/collision
   monitor pipeline (ten je na zdroji lokalizácie úplne nezávislý, netreba
   ho meniť).
7. **Offline nástroj** na zarovnanie GLIM mapy do rovnakého ENU rámca, aký
   používa `fusion_graph` (dock = (0,0), sever = +y).

## Postup nasadenia

0. **Zarovnaj GLIM mapu** (pozri `tools/align_glim_map.py`) — jednorazovo,
   pred prvým použitím. `fusion_graph`'s `map` frame je ENU okolo
   `datum_lat`/`datum_lon` (`x`=východ, `y`=sever, meter). Vaša GLIM
   `.ply` musí byť v **tom istom** rámci (translácia máte zadarmo, ak
   mapovanie štartuje na dokovačke — rotáciu treba doriešiť, pozri nižšie).
1. Aplikuj patch/nové súbory z `fusion_graph_patch/` do svojho forku.
2. Priprav `lidar_localization_ros2` launch (pozri `bringup/`), nasmerovaný
   na zarovnanú mapu, `frame_id=map`, remapnutý tak aby vydával
   `map→odom` (Nav2 mode) — **NEpouž ich Nav2 launch celý**, len samotný
   lokalizačný uzol, `odom→base_footprint` naďalej dáva `fusion_graph`.

   **POZOR — kolízia TF**: `fusion_graph` dnes publikuje `map→odom` AJ
   `odom→base_footprint` sám (nahradil starý dvojitý EKF). Aby ste sa
   vyhli dvom publisherom `map→odom` naraz, spustite
   `lidar_localization_ros2` v režime, kde **nepublikuje TF vôbec** — len
   topic `/pcl_pose` — a necháte `fusion_graph` ako jediného vlastníka TF
   stromu. Toto je čistejšie než ich TF a je presne to, na čo slúži nový
   kanál nižšie. Over v ich launch/param súbore flag na vypnutie TF
   broadcastu (`publish_tf`/`broadcast_tf` a podobne — názov si over v ich
   `nav2_lidar_localization.launch.py`/parametroch, keďže to je časť, ktorú
   ja odtiaľto nemám k dispozícii na kontrolu naživo).
3. Zbuilduj, spusti, over `/alignment_status` a `/fusion_graph/diagnostics`.
4. Prepni: `ros2 param set /fusion_graph_node primary_localization_source lidar`.

## Čo NEMENIŤ

- Nav2 (nezmenené, presne ako chceš).
- `nav2_params_lidar.yaml` costmap/collision pipeline (`/scan_costmap`,
  `/scan_collision`) — nezávislé od zdroja lokalizácie, funguje ďalej.
  Jediné, čo tam pribúda, je **pointcloud_to_laserscan** most z VLP16 3D
  mračna na 2D `/scan` (pôvodne to dávalo 2D LD19 priamo) — pozri
  `bringup/pointcloud_to_laserscan.launch.py`.
- `use_lidar_map_anchor` necháme `false` — je to nezávislý fallback
  subsystém, môžete ho neskôr zapnúť ako "fallback pri výpadku aj LiDARu aj
  GPS" (dvojitá poistka), ale nie je to potrebné na dnešný cieľ.
