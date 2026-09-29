# Kalibrácia LiDAR mapy (lidar_map → map)

GLIM mapa má vlastný lokálny rámec (počiatok a natočenie podľa miesta, kde
začalo mapovanie). `fusion_graph`, zóny v GUI a dok sú v ENU rámci okolo
datumu (`x` = východ, `y` = sever). `calibrate_lidar_map_node` z krátkej jazdy
pod RTK-Fixed vypočíta tuhú 2D transformáciu

    p_map = R(yaw) · p_lidar_map + (x, y)

a `fusion_graph` ňou prepočítava `/pcl_pose` pred fúziou.

## Predpoklady

- `lidar_localization_ros2` beží s `global_frame_id:=lidar_map`
  (`/pcl_pose` má `frame_id: lidar_map`) a sleduje (`/alignment_status` = healthy).
  Prvá inicializácia sa robí ručne cez `/initialpose` vo frame `lidar_map`.
- GPS je v RTK-Fixed.
- `lidar_enabled: true` (node sa spúšťa len s LiDARom).

## Postup

```bash
ros2 service call /calibrate_lidar_map_node/start std_srvs/srv/Trigger
ros2 topic echo /calibrate_lidar_map_node/status   # JSON s priebehom
```

Jazdi pomaly (< 0,4 m/s) čo najväčšiu časť záhrady — veľké „L“ alebo osmičku,
nie len rovnú čiaru. Kalibrácia skončí sama (`"state":"succeeded"`), keď:

- má ≥ 300 párov (RTK fix ↔ LiDAR pozícia antény),
- trajektória má rozptyl ≥ 2 m v hlavnom a ≥ 1 m vo vedľajšom smere (std),
- RMS rezíduí ≤ 5 cm,
- výsledok sa 20 s nemení (< 0,2°, < 2 cm).

Výsledok sa zapíše do `/ros2_ws/maps/lidar_map_calibration.yaml` a hneď sa
nastaví do `fusion_graph_node` (bez reštartu). Potom:

```bash
ros2 param set /fusion_graph_node primary_localization_source lidar
```

Zrušenie: `ros2 service call /calibrate_lidar_map_node/cancel std_srvs/srv/Trigger`.
Zmazanie kalibrácie: zmaž súbor a reštartuj stack.

## Čo sa kontroluje

- GPS: RTK-Fixed z `/gps/status` a `horizontal_accuracy` ≤ 3 cm (fallback NavSatFix).
- LiDAR: `/alignment_status` healthy, `/pcl_pose` interpolovaný na čas fixu (medzera ≤ 0,5 s; fix čaká na LiDAR pózu max. 3 s).
- Pohyb: rýchlosť ≤ 0,5 m/s, yaw rate ≤ 0,6 rad/s, rozostup párov ≥ 5 cm.
- Rameno antény (`gps_x/gps_y`) sa otáča yaw-om z LiDARu, nie z `fusion_graph`,
  takže kalibrácia platí aj keď je zdroj `lidar`.
- Dôvody zahodenia párov sú v `status.rejected`.

## Platnosť

Kalibrácia platí pre danú mapu a datum. Pri zmene datumu ju
`fusion_graph.launch.py` zahodí (vypíše varovanie). Pri novej GLIM mape ju
treba spustiť znova.

## Štart bez ručného zadávania pozície

Po kalibrácii netreba posielať `set_pose` ani `/initialpose`:

- **`lidar_auto_seed`** — keď `lidar_localization` nesleduje (`/alignment_status`
  nie je `healthy` dlhšie ako 3 s) a graf je inicializovaný (dok, GPS), `fusion_graph`
  mu pošle svoju pozíciu ako `/initialpose` vo frame `lidar_map` (najviac raz za 10 s).
- **`lidar_bootstrap_from_pose`** — keď je zdroj `lidar` a graf ešte nie je
  inicializovaný (bez GPS, mimo doku), prvá zdravá `/pcl_pose` sa použije ako
  počiatočná pozícia grafu.
- **Zdroj pri štarte:** do nainštalovaného `mowgli_robot.yaml` (sekcia robota) pridaj
  `primary_localization_source: lidar`. Bez platnej kalibrácie node zostane na `gps`.

Jediný prípad, kde treba zasiahnuť ručne: štart mimo doku **a** bez GPS **a**
`lidar_localization` ešte nesleduje — vtedy raz `/initialpose` (frame `lidar_map`).

## V GUI

- **Diagnostika → Kalibrácia → Kalibrácia LiDAR mapy**: tlačidlá Štart/Zrušiť
  (volajú `~/start` a `~/cancel`), živý priebeh z `~/status` (páry, rozptyl
  trajektórie, RTK, stav LiDARu, dôvody zahodenia) a uložený výsledok zo
  `lidar_map_calibration.yaml` (vrátane varovania pri inom datume). Počas zberu
  jazdi ručne cez mapu (manuálny režim).
- **Prepínač GPS ↔ LiDAR** (dashboard → Kontrola stavu, Nastavenia →
  Lokalizácia, a pri kalibrácii): nastaví `primary_localization_source` na
  `fusion_graph_node` za behu a uloží ho do `mowgli_robot.yaml`, takže platí aj
  po reštarte. Bez platnej kalibrácie `fusion_graph` LiDAR odmietne a GUI to
  ohlási (nič sa neuloží). Zobrazuje sa len s `lidar_enabled: true`.

## Mapovanie GLIM z GUI

**Diagnostika → Kalibrácia → Mapovanie GLIM** ovláda `glim_supervisor` v GLIM
kontajneri (repo `3d_mowgli_slam_stack`, `glim/supervisor/`). Kontajner beží
stále a GLIM sa spúšťa len na požiadanie:

- *Začať mapovanie* / *Ukončiť a uložiť*: každé mapovanie ide do nového
  priečinka `glim_sessions/mapping_<dátum_čas>/`, nič sa neprepisuje.
- *Otvoriť vo vieweri*: offline viewer sa otvorí rovno s danou reláciou
  (na displeji robota, cez vzdialenú plochu). Spojené mapy ulož cez *Save Map*
  do nového priečinka v `/glim/sessions`.
- *Použiť ako mapu*: export relácie do `garden_map.ply`, ktorú číta
  `lidar_localization`. Predchádzajúca mapa ostane ako záloha. Potom reštartuj
  `lidar_localization` a sprav kalibráciu LiDAR mapy znova.

Lokalizátor sa pri mapovaní nevypína. Podmienky (GPS, dok) sa nekontrolujú.
## Prevádzka bez GPS (LiDAR ako primárny zdroj)

`fusion_graph` publikuje `/fusion_graph/primary_source` (`gps`|`lidar`, latched) a
každú LiDAR pózu, ktorá prešla jeho kontrolami, ako `/fusion_graph/lidar_pose`
(frame `map`). Keď je zdroj `lidar` a táto póza je čerstvá (< 1 s), berie sa to
ako „lokalizácia v poriadku“ všade, kde MowgliNext inak žiada GPS:

| Miesto | Pri zdroji `lidar` |
|---|---|
| `LocalizationGuard` | stráži LiDAR (výpadok > 3 s = pauza), funguje aj bez GPS prijímača; σ-backstop ostáva |
| `PreFlightCheck` | GPS fix nie je potrebný |
| `WaitForGpsFix` | hneď pokračuje |
| `SeedYawFromMotion` | preskočí sa (smer pozná LiDAR) |
| `CalibrateHeadingFromUndock` | len kontrola „zaseknutý v doku“ podľa LiDAR posunu, žiadne GPS spresnenie |
| `GPSModeSelector` | `IsLocalizationPrecise` = RTK Fixed **alebo** LiDAR OK |
| `gps_dock_detection_node` | poloha robota pre dokovanie z LiDARu namiesto RTK |

Pri zdroji `gps` sa správanie nemení. Detektor zabárania (hardware_bridge) bez RTK
naďalej stojí — chráni len anti-dig vo firmvéri.
